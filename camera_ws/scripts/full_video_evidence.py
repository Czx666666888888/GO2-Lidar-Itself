"""Collect finalized wrist recordings without controlling cameras or the arm."""
import json
from pathlib import Path
import shutil
import subprocess


def save_front_evidence(folder, front):
    """Index front video even when docking fails and no grasp is requested."""
    report = dict(front=front, wrist=dict(status='NOT STARTED', reason='No grasp requested yet'))
    (Path(folder)/'recordings.json').write_text(json.dumps(report, indent=2)+'\n')


def save_video_evidence(args, folder, front, previous, arm_command):
    """Retain remote evidence on transfer failures and report them explicitly."""
    folder = Path(folder)
    report = dict(front=front, wrist=dict(status='NOT VERIFIED'))
    try:
        wrist = arm_command(args, 'status').get('last_recording')
        if not wrist or wrist.get('directory') == (previous or {}).get('directory'):
            report['wrist'] = dict(status='NOT STARTED', reason='No new grasp recording')
        else:
            report['wrist'] = dict(recording=wrist, status='REMOTE_ONLY')
            remote = Path(wrist['directory'])
            expected = Path('/home/unitree/s1-arm-control-v2/data/grasp_videos')
            if remote.parent != expected or not remote.name.startswith('grasp_'):
                raise RuntimeError('Unexpected wrist recording path; preserve remote source')
            if wrist.get('finalized') is not True:
                raise RuntimeError('Wrist recording not finalized; remote source retained')
            destination = folder/'wrist'
            destination.mkdir(exist_ok=True)
            if args.arm_host == 'local':
                shutil.copytree(remote, destination/remote.name)
            else:
                subprocess.run(['scp', '-r', '-i', args.ssh_key, '-o', 'BatchMode=yes',
                                '-o', 'ConnectTimeout=5',
                                args.arm_host+':'+str(remote), str(destination)],
                               timeout=120, check=True)
            report['wrist'].update(status='COPIED', local_directory=str(destination/remote.name))
    except Exception as exc:
        report['wrist']['error'] = str(exc)
        print('WRIST_VIDEO_EVIDENCE_ERROR '+str(exc), flush=True)
    (folder/'recordings.json').write_text(json.dumps(report, indent=2)+'\n')
    print('FULL_VIDEO_EVIDENCE '+str(folder/'recordings.json'), flush=True)
