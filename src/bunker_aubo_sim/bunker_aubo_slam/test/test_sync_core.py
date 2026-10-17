from bunker_aubo_slam.sync_core import BLANK, SyncCore


def feed(core, cam, stamp, now):
    return core.add(cam, stamp, f'c{cam}@{stamp:.2f}', now)


def test_waits_until_every_camera_was_seen():
    core = SyncCore(2)
    assert feed(core, 0, 1.0, 0.0) == []
    out = feed(core, 1, 1.0, 0.01)
    assert out == [(1.0, ['c0@1.00', 'c1@1.00'])]


def test_pairs_closest_stamp_within_tolerance():
    core = SyncCore(2, max_dt=0.05)
    feed(core, 1, 0.98, 0.0)
    feed(core, 1, 1.03, 0.0)
    out = feed(core, 0, 1.0, 0.01)
    assert out == [(1.0, ['c0@1.00', 'c1@0.98'])]


def test_partner_arriving_late_is_waited_for():
    core = SyncCore(2, max_dt=0.05, pending_timeout=0.3)
    feed(core, 0, 1.0, 0.0)
    feed(core, 1, 1.0, 0.0)          # first pair
    assert feed(core, 0, 1.1, 0.1) == []
    assert feed(core, 1, 1.1, 0.15) == [(1.1, ['c0@1.10', 'c1@1.10'])]


def test_dead_camera_replaced_by_blank():
    core = SyncCore(2, camera_timeout=0.5)
    feed(core, 0, 1.0, 0.0)
    feed(core, 1, 1.0, 0.0)
    out = []
    t = 0.0
    for k in range(1, 11):
        t = 0.1 * k
        out += feed(core, 0, 1.0 + 0.1 * k, t)
        out += core.flush(t)
    blanks = [g for _, g in out if g[1] is BLANK]
    assert blanks, 'SLAM input must continue while a camera is down'
    assert all(g[0] is not BLANK for _, g in out)
    stamps = [s for s, _ in out]
    assert stamps == sorted(stamps) and len(stamps) == len(set(stamps))


def test_reference_switches_when_first_camera_dies():
    core = SyncCore(2, camera_timeout=0.5)
    feed(core, 0, 1.0, 0.0)
    feed(core, 1, 1.0, 0.0)
    out = []
    for k in range(1, 11):
        t = 0.1 * k
        out += feed(core, 1, 1.0 + 0.1 * k, t)
    late = [g for s, g in out if s > 1.6]
    assert late and all(g[0] is BLANK and g[1] != BLANK for g in late)


def test_recovery_after_dropout():
    core = SyncCore(2, camera_timeout=0.5)
    feed(core, 0, 1.0, 0.0)
    feed(core, 1, 1.0, 0.0)
    for k in range(1, 8):
        feed(core, 0, 1.0 + 0.1 * k, 0.1 * k)
    feed(core, 1, 1.8, 0.8)
    out = feed(core, 0, 1.8, 0.81)
    assert out[-1] == (1.8, ['c0@1.80', 'c1@1.80'])
