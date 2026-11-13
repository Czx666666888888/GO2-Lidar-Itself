"""A fixed, IMU-assisted local clock mapping; not absolute clock synchronization."""
from collections import deque

import numpy as np


class ImuTimeMapping:
    """Estimate b once from low-latency IMU samples, then keep t_local=t_sensor+b."""

    def __init__(self, window_sec=2.0, min_samples=200, percentile=5.0,
                 max_spread_sec=0.020, explicit_offset_sec=None):
        if (not np.isfinite(window_sec) or window_sec <= 0 or min_samples < 2
                or not np.isfinite(max_spread_sec) or max_spread_sec <= 0
                or not 0 <= percentile <= 50):
            raise ValueError('invalid IMU mapping window/sample count/percentile')
        self.window_ns = round(window_sec*1e9)
        self.min_samples = min_samples
        self.percentile = percentile
        self.max_spread_ns = round(max_spread_sec*1e9)
        self.offset_ns = (None if explicit_offset_sec is None
                          else round(explicit_offset_sec*1e9))
        self.samples = deque()
        self.started_ns = None
        self.last_clock = None
        self.last_imu_ns = None
        self.last_output = {}
        self.dropped = {}
        self.fault = ''
        self.metadata_samples = 0
        self.mapped_end_future_ns = None

    @property
    def ready(self):
        return self.offset_ns is not None and not self.fault

    def observe_imu(self, stamp_ns, source_ns, receive_ns, monotonic_ns):
        """Use callback time as an approximate observation, never a per-message replacement."""
        if self.fault or stamp_ns <= 0:
            return False
        if self.last_clock:
            ros_delta = receive_ns-self.last_clock[0]
            mono_delta = monotonic_ns-self.last_clock[1]
            if mono_delta < 0 or abs(ros_delta-mono_delta) > 50_000_000:
                self.fault = 'local clock stepped; restart the read-only localization session'
                return False
        self.last_clock = (receive_ns, monotonic_ns)
        if self.last_imu_ns is not None and stamp_ns <= self.last_imu_ns:
            self.dropped['raw_imu_nonadvancing'] = self.dropped.get('raw_imu_nonadvancing', 0)+1
            if self.last_imu_ns-stamp_ns >= 1_000_000_000:
                self.fault = 'sensor clock moved backwards; no automatic relock during navigation'
            return False
        self.last_imu_ns = stamp_ns
        # DDS time verifies the source/header relationship and exposes producer
        # delay. It is not an independent UTC reference. Missing metadata is
        # allowed; receipt minus raw IMU stamp is still an approximate mapping.
        publication_delay = None
        if source_ns:
            publication_delay = source_ns-stamp_ns
            if not -1_000_000 <= publication_delay <= 20_000_000:
                self.dropped['dds_header_mismatch'] = self.dropped.get('dds_header_mismatch', 0)+1
                return False
            self.metadata_samples += 1
        if self.started_ns is None:
            self.started_ns = monotonic_ns
        self.samples.append((monotonic_ns, receive_ns-stamp_ns, publication_delay))
        while self.samples and monotonic_ns-self.samples[0][0] > self.window_ns:
            self.samples.popleft()
        if self.offset_ns is None and monotonic_ns-self.started_ns >= self.window_ns:
            if len(self.samples) >= self.min_samples:
                values = [r[1] for r in self.samples]
                low, high = np.percentile(values, [self.percentile, 95])
                if high-low <= self.max_spread_ns:
                    self.offset_ns = round(float(low))
        return self.ready

    def map_stamp(self, kind, stamp_ns, receive_ns, scan_duration_ns=0):
        if not self.ready:
            return None
        mapped = stamp_ns+self.offset_ns
        previous = self.last_output.get(kind)
        if previous is not None and mapped <= previous:
            self.dropped[kind+'_nonadvancing'] = self.dropped.get(kind+'_nonadvancing', 0)+1
            return None
        end = mapped+scan_duration_ns
        if kind == 'cloud':
            self.mapped_end_future_ns = end-receive_ns
        if mapped <= 0 or end-receive_ns > 100_000_000 or receive_ns-mapped > 1_000_000_000:
            self.fault = kind+' mapped time invalid (including scan end); restart localization'
            return None
        self.last_output[kind] = mapped
        return mapped

    def status(self):
        values = [r[1] for r in self.samples]
        delays = [r[2] for r in self.samples if r[2] is not None]
        low = round(float(np.percentile(values, self.percentile))) if values else None
        return dict(ready=self.ready, fault=self.fault, scale=1.0,
                    offset_ns=self.offset_ns,
                    offset_sec=None if self.offset_ns is None else self.offset_ns/1e9,
                    window_samples=len(values), window_sec=self.window_ns/1e9,
                    percentile=self.percentile, candidate_low_offset_ns=low,
                    residual_offset_sec=None if low is None or self.offset_ns is None
                    else (low-self.offset_ns)/1e9,
                    spread_sec=None if not values else
                    float(np.percentile(values, 95)-np.percentile(values, self.percentile))/1e9,
                    dds_metadata_samples=self.metadata_samples,
                    producer_delay_median_sec=None if not delays else float(np.median(delays))/1e9,
                    mapped_cloud_end_future_sec=None if self.mapped_end_future_ns is None
                    else self.mapped_end_future_ns/1e9,
                    dropped=dict(self.dropped),
                    absolute_clock_synchronization=False,
                    limitation='One-way transport/dispatch delay is not identifiable separately from clock offset')
