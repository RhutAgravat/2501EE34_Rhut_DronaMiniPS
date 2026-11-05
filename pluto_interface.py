"""Exclusive RC command route to the real simulator."""
import numpy as np
class PlutoInterface:
    def __init__(self, simulator, failsafe_s=1.0):
        self.sim=simulator; self.armed=set(); self.failsafe_s=float(failsafe_s); self.last_command={}
    def arm(self, drone_id): self.armed.add(drone_id); return True
    def disarm(self, drone_id): self.armed.discard(drone_id); return True
    def send_rc(self, drone_id, rc):
        out=[int(np.clip(v,1000,2000)) for v in rc]
        if drone_id in self.armed:
            self.sim.set_rc(drone_id, out); self.last_command[drone_id]=float(getattr(self.sim, 't', 0.0))
        return out

    def watchdog(self):
        """Firmware-style command-loss failsafe: land after one second."""
        now=float(getattr(self.sim, 't', 0.0))
        landed=[]
        for drone_id in sorted(self.armed):
            if now-self.last_command.get(drone_id, -float('inf')) >= self.failsafe_s:
                self.sim.set_rc(drone_id, [1500, 1500, 1500, 1000]); landed.append(drone_id)
        return landed
