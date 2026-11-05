"""Execute F1/F3 fault drills with the real observation/localization path."""
from pathlib import Path
import json, numpy as np
from config import load_config
from camera import OverheadCamera, Observation
from localization import Localizer
from safety import SafetySupervisor
from pluto_interface import PlutoInterface
from simulator import SwarmSim

def main(root='.'):
    cfg=load_config(Path(root)/'config.yaml'); cam=OverheadCamera(cfg,seed=3); loc=Localizer(cfg,4); safe=SafetySupervisor(cfg)
    pos=np.array([[-.6,0,1.],[ -.2,0,1.],[.2,0,1.],[.6,0,1.]])
    f1=[]
    for k in range(30):
        t=k*.04; obs=cam.capture(pos,t,hidden=(2,) if t<.4 else ()); e=loc.update(obs,t); f1.append(safe.check(e,t))
    jump=[Observation(1,(640,360),np.array([0.,0.,1.]),1.0)]
    loc.update(jump,1.0); loc.update([Observation(1,(640,360),np.array([1.,0.,1.]),1.04)],1.04)
    result={'F1_hidden_drone':2,'F1_actions':sum(bool(x) for x in f1),'F3_rejected_outliers':loc.rejected,'F3_expected':'reject 1 m jump'}
    # Establish a delayed-camera baseline before hiding drone 3 for exactly 1.2 s.
    f2_cam=OverheadCamera(cfg,seed=4); f2_loc=Localizer(cfg,4); f2_safe=SafetySupervisor(cfg); f2_times=[]; baseline_visible=False
    for k in range(80):
        t=k*.04; hidden=(3,) if 0.4 <= t < 1.6 else (); e=f2_loc.update(f2_cam.capture(pos,t,hidden=hidden),t)
        if t < .4 and e[3]['status'] == 'valid': baseline_visible=True
        # Reset event history at drill start so the logged stale/lost event is the
        # deliberate F2 fault, not camera startup latency.
        if abs(t - .4) < 1e-9: f2_safe=SafetySupervisor(cfg)
        actions=f2_safe.check(e,t)
        if t >= .4 and e[3]['status'] == 'lost': f2_times.append(t)
    sim=SwarmSim(1,False,cfg); radio=PlutoInterface(sim); radio.arm(0); radio.send_rc(0,[1500,1500,1500,1870]); sim.step(241); f4=radio.watchdog()
    result.update({'F2_hidden_1_2s': {'drone_id': 3, 'baseline_visible': baseline_visible, 'hidden_start_s': .4, 'hidden_end_s': 1.6, 'lost_observations': len(f2_times), 'first_lost_s': f2_times[0] if f2_times else None, 'safety_events': f2_safe.events},
                   'F4_firmware_failsafe': {'after_1s': f4 == [0], 'landed_ids': f4}})
    sim.close()
    Path(root,'logs','fault_drills.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
