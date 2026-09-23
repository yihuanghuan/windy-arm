from real_arm_readonly.read_guard import ReadGuard


def test_spike_warns_but_sustained_velocity_aborts():
    g=ReadGuard()
    assert g.update(0x21,[0,0,0,0,.159,0],0) is None
    assert g.warnings[4] == 1
    assert g.update(0x21,[0]*6,.005) is None
    for i in range(4): assert g.update(0x21,[0,0,0,0,.2,0],.01+i*.005) is None
    assert 'velocity' in g.update(0x21,[0,0,0,0,.2,0],.03)


def test_position_jump_and_consistent_motion():
    g=ReadGuard(); g.update(0x20,[0]*6,0)
    assert 'jump' in g.update(0x20,[.1,0,0,0,0,0],.005)
    g=ReadGuard()
    reason=None
    for i in range(13):
        g.update(0x21,[.1,0,0,0,0,0],i*.005)
        reason=g.update(0x20,[i*.0005,0,0,0,0,0],i*.005)
        if reason: break
    assert 'consistently' in reason


def test_quantized_position_jitter_and_nonfinite():
    g=ReadGuard()
    for i in range(100):
        assert g.update(0x20,[0,0,0,.000383*(i%2),0,0],i*.005) is None
    assert 'nonfinite' in g.update(0x21,[float('nan')]*6,1)
