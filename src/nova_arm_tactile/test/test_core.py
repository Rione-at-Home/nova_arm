"""Parser, region/mask, features and the serial channel, fed from the demo stream (no ROS)."""
import time

import numpy as np
import pytest

from nova_arm_tactile.core import (COLS, MAGIC, ROWS, DemoStream, FingerChannel, FrameParser,
                                   compute_features, parse_mask, parse_region)


def _frame(v=7):
    return MAGIC + bytes([v]) * (ROWS * COLS)


def test_parser_needs_next_header_and_rejects_shifted_frames():
    p = FrameParser()
    assert p.feed(_frame(1)) == []                      # cannot validate yet
    out = p.feed(_frame(2))
    assert len(out) == 1 and out[0].shape == (ROWS, COLS) and out[0][0, 0] == 1
    broken = _frame(3)[:-1]                             # one byte lost
    out = p.feed(broken + _frame(4) + _frame(5))
    assert [int(f[0, 0]) for f in out] == [2, 4]        # 3 rejected, nothing shifted
    assert p.bad_frames >= 1


def test_region_and_mask():
    outside = parse_region("2:14,0:32")
    assert outside[0].all() and outside[15].all() and not outside[2:14].any()
    assert not parse_region("").any()
    with pytest.raises(ValueError):
        parse_region("0:20,0:32")
    m = parse_mask("0,1; 15,31")
    assert m[0, 1] and m[15, 31] and m.sum() == 2


def test_features_ignore_masked_region():
    base = np.full((ROWS, COLS), 5, np.float32)
    raw = base.copy()
    raw[0, 5] = 200                                     # outside the region
    raw[8, 10:13] = 100
    f = compute_features(raw, base, parse_region("2:14,0:32"), 15.0, 2)
    assert f.in_contact and f.contact_taxels == 3
    assert f.centroid_row == pytest.approx(8.0) and f.centroid_col == pytest.approx(11.0)


def test_channel_calibrates_and_reports_health_with_driven_press():
    press = {"amp": 0.0}
    stream = DemoStream(rate_hz=200.0, press=lambda: (press["amp"], 8.0, 16.0))
    ch = FingerChannel("right", "demo", baseline_frames=10, opener=lambda: stream,
                       log=lambda _m: None)
    ch.start()
    try:
        t0 = time.time()
        while not ch.info()["calibrated"] and time.time() - t0 < 3:
            time.sleep(0.02)
        info = ch.info()
        assert info["calibrated"] and info["connected"] and info["connect_attempts"] == 1
        ch.drain()
        press["amp"] = 120.0
        time.sleep(0.2)
        frames = ch.drain()
        assert frames and frames[-1].feat.in_contact
        assert frames[-1].feat.centroid_row == pytest.approx(8.0, abs=0.5)
        press["amp"] = 0.0
        time.sleep(0.2)
        assert not ch.drain()[-1].feat.in_contact
    finally:
        ch.stop()
        ch.join(1.0)


def test_missing_port_is_retried_not_fatal():
    def boom():
        raise OSError("No such file or directory: '/dev/tactile_left'")
    ch = FingerChannel("left", "/dev/tactile_left", opener=boom, log=lambda _m: None)
    ch.max_backoff = 0.05
    ch.start()
    time.sleep(0.4)
    info = ch.info()
    ch.stop()
    ch.join(1.0)
    assert not info["connected"] and info["connect_attempts"] >= 3
    assert "No such file" in info["last_error"]