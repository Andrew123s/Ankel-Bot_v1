from ankelbot.tracker import COLLECTABLE, OVERSIZE, UNKNOWN, Measurement, Tracker


def feed(tr, p, n, **kw):
    rows = []
    for k in range(n):
        rows = tr.update(k * p.dt, 0.0, 0.0, 0.0, [Measurement(**kw)])
    return rows


def test_needs_several_observations_before_deciding(p):
    tr = Tracker(p)
    rows = feed(tr, p, 2, lat=0.2, fwd=4.0, width=0.15)
    assert rows[0][4] == UNKNOWN
    rows = feed(tr, p, 3, lat=0.2, fwd=4.0, width=0.15)
    assert len(rows) == 1 and rows[0][4] == COLLECTABLE


def test_wide_object_is_oversize(p):
    rows = feed(Tracker(p), p, 5, lat=-0.5, fwd=5.0, width=0.8)
    assert rows[0][4] == OVERSIZE


def test_reject_class_overrides_narrow_width(p):
    # a log seen end-on looks narrow; its class must still reject it
    rows = feed(Tracker(p), p, 5, lat=0.0, fwd=5.0, width=0.12, over_class=True)
    assert rows[0][4] == OVERSIZE


def test_one_outlier_does_not_flip_the_decision(p):
    tr = Tracker(p)
    feed(tr, p, 6, lat=0.0, fwd=4.0, width=0.15)
    rows = tr.update(0.7, 0.0, 0.0, 0.0, [Measurement(0.0, 4.0, 1.4)])
    assert rows[0][4] == COLLECTABLE


def test_truncated_box_cannot_shrink_the_estimate(p):
    tr = Tracker(p)
    feed(tr, p, 5, lat=0.0, fwd=3.0, width=0.9)
    for k in range(20):
        rows = tr.update(1 + k * p.dt, 0.0, 0.0, 0.0, [Measurement(0.0, 3.0, 0.2, truncated=True)])
    assert rows[0][4] == OVERSIZE


def test_two_objects_stay_separate_and_are_remembered(p):
    tr = Tracker(p)
    for k in range(5):
        rows = tr.update(k * p.dt, 0, 0, 0, [Measurement(-1.0, 4.0, 0.1), Measurement(1.0, 4.0, 0.7)])
    assert len(rows) == 2
    rows = tr.update(3.0, 0, 0, 0, [])                 # out of view, still remembered
    assert len(rows) == 2
    rows = tr.update(3.0 + p.dec.t_keep + 1, 0, 0, 0, [])
    assert rows == []
