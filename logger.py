"""CSV and PNG mission artifact writer."""
from pathlib import Path
import csv
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
class MissionLogger:
    def __init__(self,root='logs',plots='plots'): self.root=Path(root); self.plots=Path(plots); self.root.mkdir(exist_ok=True); self.plots.mkdir(exist_ok=True); self.rows=[]
    def add(self,row): self.rows.append(row)
    def save(self,name):
        if not self.rows:return None
        path=self.root/f'{name}.csv'
        with path.open('w',newline='') as f: w=csv.DictWriter(f,fieldnames=self.rows[0].keys()); w.writeheader(); w.writerows(self.rows)
        fig,ax=plt.subplots(); ax.plot([r['time'] for r in self.rows],[r.get('error',0) for r in self.rows]); ax.set(xlabel='time (s)',ylabel='error (m)'); fig.savefig(self.plots/f'{name}.png'); plt.close(fig); return path
