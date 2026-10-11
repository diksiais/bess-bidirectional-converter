"""Step 5: switched (non-averaged) boost converter with inductor DCR and capacitor ESR,
PI loop sampled once per switching period. Exact per-interval integration (matrix exponential)."""
import numpy as np, warnings; warnings.filterwarnings("ignore")
from scipy.linalg import expm
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from params import *
from pathlib import Path
FIG = Path(__file__).resolve().parent.parent / 'docs' / 'figures'; FIG.mkdir(parents=True, exist_ok=True)

import converter as avg

fsw = 50e3; T = 1/fsw
rL, rc = 20e-3, 20e-3          # inductor DCR, capacitor ESR (assumed)
D_MIN, D_MAX = 0.05, 0.90

def zmats(R):
    k = R/(R+rc); a = 1/((R+rc)*C)
    on  = np.array([[-rL/L, 0, Vin/L],[0, -a, 0],[0,0,0]])
    off = np.array([[-(rL+k*rc)/L, -k/L, Vin/L],[R/((R+rc)*C), -a, 0],[0,0,0]])
    return on, off, k

def run(Kp=Kp, Ki=Ki, t_step=0.3, R1=9.6, t_end=0.6, windows=((0.25,),(0.55,))):
    on_cache = {}
    iL, vC = Vref/(R0*(1-D0)), Vref
    x = np.array([iL, vC, 1.0]); xi = 0.0
    N = int(round(t_end/T)); rec = []; wf = []
    for n in range(N):
        t = n*T; R = R0 if t < t_step else R1
        Zon, Zoff, k = zmats(R)
        vs = k*(x[1]+rc*x[0])                  # sample just before turn-on
        e = Vref - vs
        d_unsat = D0 + Kp*e + Ki*(xi + e*T)
        d = min(max(d_unsat, D_MIN), D_MAX)
        if D_MIN < d_unsat < D_MAX: xi += e*T  # conditional integration (anti-windup)
        rec.append((t, vs, d, x[0]))
        inwin = any(w[0] <= t < w[0]+5*T for w in windows)
        if inwin:
            for tau in np.linspace(0, d*T, 12, endpoint=False):
                y = expm(Zon*tau)@x; wf.append((t+tau, y[0], k*y[1]))
        x = expm(Zon*(d*T))@x
        if inwin:
            for tau in np.linspace(0, (1-d)*T, 12, endpoint=False):
                y = expm(Zoff*tau)@x; wf.append((t+d*T+tau, y[0], k*(y[1]+rc*y[0])))
        x = expm(Zoff*((1-d)*T))@x
    return np.array(rec), np.array(wf)

if __name__ == "__main__":
    rec, wf = run()
    t, vs, d, iL0 = rec.T
    # metrics
    i = t > 0.3; dip = Vref - vs[i].min()
    out = np.where((abs(vs-Vref) > 0.02*Vref) & i)[0]
    ts = (t[out[-1]]-0.3)*1e3 if len(out) else 0
    pre = (wf[:,0] >= 0.25) & (wf[:,0] < 0.25+5*T); post = (wf[:,0] >= 0.55)
    rip = lambda m, col: wf[m, col].max()-wf[m, col].min()
    print(f"load-step dip {dip:.2f} V, 2% settling {ts:.0f} ms (sampled Vout)")
    print(f"nominal : iL ripple {rip(pre,1):.2f} A pk-pk, Vout ripple {rip(pre,2):.2f} V pk-pk, mean Vout {wf[pre,2].mean():.2f} V, min iL {wf[pre,1].min():.2f} A")
    print(f"post-step: iL ripple {rip(post,1):.2f} A pk-pk, Vout ripple {rip(post,2):.2f} V pk-pk, mean Vout {wf[post,2].mean():.2f} V, min iL {wf[post,1].min():.2f} A")
    print(f"duty before step {d[(t>0.25)&(t<0.3)].mean():.4f}, after {d[t>0.55].mean():.4f}")
    # ideal averaged model for comparison
    sa, da = avg.closed_loop(Kp, Ki, T=0.6)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    ax[0,0].plot(sa.t*1e3, sa.y[1], label="averaged (ideal)"); ax[0,0].plot(t*1e3, vs, lw=.8, label="switched (sampled Vout)")
    ax[0,0].set(xlim=(250, 450), ylim=(88, 100), title="Load step 14.4 -> 9.6 ohm at 300 ms", xlabel="ms", ylabel="Vbus (V)"); ax[0,0].legend()
    ax[0,1].plot(t*1e3, d); ax[0,1].set(xlim=(250,450), title="Duty command", xlabel="ms", ylabel="d")
    w = wf[pre]; ax[1,0].plot((w[:,0]-0.25)*1e6, w[:,1]); ax[1,0].set(title="Inductor current ripple (nominal load)", xlabel="us", ylabel="iL (A)")
    ax[1,1].plot((w[:,0]-0.25)*1e6, w[:,2]); ax[1,1].set(title="Output voltage ripple incl. ESR step (nominal load)", xlabel="us", ylabel="Vout (V)")
    plt.tight_layout(); plt.savefig(FIG/"switched_overview.png", dpi=130)
