from go2_science_perception.statistics import StreamStats


def test_frequency_uses_inter_message_intervals():
    stats = StreamStats()
    stats.update(10.0)
    stats.update(10.1)
    stats.update(10.2)
    assert abs(stats.frequency_hz - 10.0) < 1e-9


def test_freshness_requires_a_recent_message():
    stats = StreamStats()
    assert not stats.is_fresh(10.0, 2.0)
    stats.update(9.0)
    assert stats.is_fresh(10.0, 2.0)
    assert not stats.is_fresh(12.0, 2.0)
