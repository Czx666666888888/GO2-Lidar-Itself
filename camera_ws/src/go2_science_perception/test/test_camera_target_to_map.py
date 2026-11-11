"""Tests for the camera-to-map transform helpers."""

import math

from builtin_interfaces.msg import Time
import pytest

from go2_science_perception.camera_target_to_map import (
    make_static_transform,
    point_minus_tf_seconds,
    quaternion_from_rpy,
    stamp_to_nanoseconds,
)


def test_positive_pitch_30_degree_quaternion():
    quaternion = quaternion_from_rpy(0.0, math.radians(30.0), 0.0)
    assert quaternion == pytest.approx(
        (0.0, math.sin(math.radians(15.0)), 0.0,
         math.cos(math.radians(15.0)))
    )


def test_static_extrinsic_targets_camera_link_not_optical_frame():
    transform = make_static_transform(
        "vehicle", "camera_link", (0.36, 0.0, 0.12),
        (0.0, 30.0, 0.0), Time(sec=12, nanosec=34)
    )
    assert transform.header.frame_id == "vehicle"
    assert transform.child_frame_id == "camera_link"
    assert transform.transform.translation.x == pytest.approx(0.36)
    assert transform.transform.translation.z == pytest.approx(0.12)
    assert transform.transform.rotation.y == pytest.approx(
        math.sin(math.radians(15.0))
    )


def test_stamp_to_nanoseconds_preserves_full_precision():
    stamp = Time(sec=12, nanosec=345678901)
    assert stamp_to_nanoseconds(stamp) == 12345678901


def test_positive_delta_means_point_is_newer_than_latest_tf():
    point_stamp = Time(sec=20, nanosec=250000000)
    tf_stamp = Time(sec=19, nanosec=900000000)
    assert point_minus_tf_seconds(point_stamp, tf_stamp) == pytest.approx(0.35)


def test_negative_delta_means_tf_is_newer_than_point():
    point_stamp = Time(sec=20, nanosec=100000000)
    tf_stamp = Time(sec=20, nanosec=175000000)
    assert point_minus_tf_seconds(point_stamp, tf_stamp) == pytest.approx(-0.075)
