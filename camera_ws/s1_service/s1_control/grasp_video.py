"""Record the wrist camera through one shared RGB-D pipeline during a grasp.

Imported lazily by the existing S1 worker, never by a second serial owner.
The upstream capture function keeps its alignment/depth/quality checks and
borrows fresh frames from this pipeline instead of opening the D455 again.
"""
import json
from pathlib import Path
import threading
import time
import uuid


_active = None


class CaptureLease:
    """Upstream's start/wait/stop interface without taking camera ownership."""

    def __init__(self, recording):
        self.recording = recording
        self.sequence = recording.sequence

    def start(self, config):
        self.sequence = self.recording.sequence
        return self.recording.profile

    def wait_for_frames(self, timeout_ms=5000):
        deadline = time.monotonic() + timeout_ms / 1000.
        owner = self.recording
        with owner.condition:
            while owner.sequence <= self.sequence:
                if owner.camera_error:
                    raise RuntimeError(owner.camera_error)
                remaining = deadline - time.monotonic()
                if remaining <= 0 or owner.stop_event.is_set():
                    raise RuntimeError('D455 shared capture timed out or stopped')
                owner.condition.wait(remaining)
            self.sequence = owner.sequence
            return owner.latest

    def stop(self):
        # Only the task recording context may stop the physical pipeline.
        pass


def capture_pipeline(rs, context, serial, width, height, fps):
    """Keep standalone capture unchanged; enforce the task's bound profile."""
    if _active is None:
        return rs.pipeline(context)
    if (str(serial), width, height, fps) != _active.settings:
        raise RuntimeError('Capture profile differs from the recorded wrist configuration')
    return CaptureLease(_active)


class GraspRecording:
    """Continuously read RGB-D; save RGB and acquisition timestamps on the host."""

    def __init__(self, folder, serial, width, height, fps):
        self.folder = Path(folder)
        self.settings = (str(serial), width, height, fps)
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.sequence = 0
        self.latest = None
        self.camera_error = None
        self.errors = []
        self.frames = 0
        self.pipeline = self.writer = self.thread = self.log = None
        self.started = False
        self.result = dict(directory=str(self.folder), serial=str(serial),
                           video=str(self.folder/'d455_raw.mp4'),
                           nominal_fps=fps, width=width, height=height,
                           started_ns=time.time_ns(), finalized=False, complete=False)

    def start(self):
        import cv2
        import pyrealsense2 as rs
        self.folder.mkdir(parents=True, exist_ok=False)
        self.log = (self.folder/'frames.jsonl').open('x', buffering=1)
        serial, width, height, fps = self.settings
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.profile = self.pipeline.start(config)
        self.started = True
        actual = self.profile.get_device().get_info(rs.camera_info.serial_number)
        if actual != serial:
            raise RuntimeError('Wrist recording serial mismatch: '+str(actual))
        self.writer = cv2.VideoWriter(self.result['video'], cv2.VideoWriter_fourcc(*'mp4v'),
                                      fps, (width, height))
        if not self.writer.isOpened():
            raise RuntimeError('Cannot open D455 MP4 writer')
        self.thread = threading.Thread(target=self._read, name='d455-grasp-video', daemon=True)
        self.thread.start()
        # Prove real frames before allowing the existing grasp callback to run.
        CaptureLease(self).wait_for_frames(5000)
        if not self.frames or self.errors:
            raise RuntimeError('Wrist recording did not receive a writable RGB frame')
        return self

    def _read(self):
        import cv2
        import numpy as np
        while not self.stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(1000)
                color = frames.get_color_frame()
                if not color or not frames.get_depth_frame():
                    raise RuntimeError('Wrist RGB/depth frame missing')
                # Deliver frames even if storage subsequently fails, preserving
                # the original grasp's measured fault/hold handling.
                if not self.errors:
                    try:
                        self.writer.write(cv2.cvtColor(np.asanyarray(color.get_data()),
                                                       cv2.COLOR_RGB2BGR))
                        self.log.write(json.dumps(dict(
                            frame_index=self.frames, host_time_ns=time.time_ns(),
                            host_monotonic=time.monotonic(),
                            camera_frame_number=color.get_frame_number(),
                            camera_timestamp_ms=color.get_timestamp(),
                            timestamp_domain=str(color.get_frame_timestamp_domain())))+'\n')
                        self.frames += 1
                    except Exception as exc:
                        self.errors.append('recording: '+str(exc))
                with self.condition:
                    self.latest = frames
                    self.sequence += 1
                    self.condition.notify_all()
            except Exception as exc:
                self.camera_error = str(exc)
                self.errors.append('camera: '+str(exc))
                with self.condition:
                    self.condition.notify_all()
                return

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3)
        if self.thread is not None and self.thread.is_alive():
            self.errors.append('camera reader did not exit; recording not finalized')
            self.result.update(errors=list(self.errors), frames=self.frames)
        else:
            if self.started:
                try:
                    self.pipeline.stop()
                except Exception as exc:
                    self.errors.append('camera stop: '+str(exc))
            if self.writer is not None:
                self.writer.release()
            if self.log is not None:
                self.log.close()
            if self.frames:
                import cv2
                probe = cv2.VideoCapture(self.result['video'])
                try:
                    readable, _ = probe.read()
                    self.result['container_frames'] = int(probe.get(cv2.CAP_PROP_FRAME_COUNT))
                    if not readable:
                        self.errors.append('Finalized MP4 has no decodable first frame')
                    if self.result['container_frames'] != self.frames:
                        self.errors.append(
                            'MP4 container frame count differs from submitted frames')
                finally:
                    probe.release()
            self.result.update(finalized=True, frames=self.frames,
                               complete=self.frames > 0 and not self.errors,
                               errors=list(self.errors), ended_ns=time.time_ns())
        if self.folder.exists():
            (self.folder/'recording.json').write_text(json.dumps(self.result, indent=2)+'\n')
        return dict(self.result)


def run_recorded_flow(flow, args, session, callback):
    """Record only an actual grasp job, including deployment and return to hold."""
    global _active
    if _active is not None:
        raise RuntimeError('A wrist recording already owns the camera')
    _, config = flow.load_alignment_config(args.config)
    loaded = flow.load_wrist_servo_config(flow.resolve_project_path(config['wrist_servo_config']))
    capture = loaded.record['capture']
    name = 'grasp_'+str(time.time_ns())+'_'+uuid.uuid4().hex[:8]
    folder = flow.PROJECT_ROOT/'data/grasp_videos'/name
    recording = GraspRecording(folder, loaded.record['camera_serial'],
                               int(capture['width']), int(capture['height']), int(capture['fps']))
    session._update(last_recording=dict(recording.result))
    try:
        recording.start()
        _active = recording
        return callback(session)
    except BaseException as exc:
        recording.result['task_error'] = str(exc)
        raise
    finally:
        _active = None
        try:
            result = recording.close()
        except Exception as exc:
            result = dict(recording.result, complete=False, finalized=False,
                          errors=recording.errors+['finalization: '+str(exc)])
        session._update(last_recording=result)
        print('D455_RECORDING '+json.dumps(result), flush=True)
