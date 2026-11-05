import numpy as np

def test_assignment_is_nontrivial():
    from planner import assign_slots, line_slots
    slots=line_slots(4,.4,1.); pos=slots[[2,0,3,1]]
    a,c=assign_slots(pos,slots)
    assert list(a)==[2,0,3,1] and c==0

def test_camera_localization_outlier_rejection():
    from config import load_config
    from camera import OverheadCamera, Observation
    from localization import Localizer
    cfg=load_config(); l=Localizer(cfg,1); l.update([Observation(0,(0,0),np.array([0.,0.,1.]),0.)],0.)
    l.update([Observation(0,(0,0),np.array([1.,0.,1.]),.04)],.04)
    assert l.rejected==1 and l.get(0,.04)['status']=='valid'

def test_safety_lost_lands():
    from config import load_config
    from safety import SafetySupervisor
    s=SafetySupervisor(load_config()); a=s.check({2:{'position':None,'status':'lost'}},1.)
    assert a[2]=='land' and s.events

def test_controller_uses_calibrated_non_1500_hover_throttle():
    from config import load_config
    from controller import CascadedPID
    cfg = load_config()
    cfg['control']['hover_rc'] = 1870
    command = CascadedPID(cfg).command(0, np.array([0., 0., 1.]), np.array([0., 0., 1.]))
    assert command[3] == 1870

