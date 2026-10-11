"""Step 6: synchronous half-bridge current control (charge/buck direction, and through zero into discharge).
Current I is positive INTO the battery. Bus is a stiff source (held by another unit while charging).
Plant: L dI/dt = d*Vbus - Vbat - Rt*I, Rt = rL + Rb.
PI zero placed at Wc/5 (NOT cancelling the plant pole) + bus-voltage feedforward d_ff = Vbat/Vbus.
(Cancelling the plant pole gives a 25 ms recovery tail after a bus sag; see WRITEUP.)"""
import numpy as np, warnings; warnings.filterwarnings("ignore")
from scipy import signal
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from params import *
from pathlib import Path
FIG = Path(__file__).resolve().parent.parent / 'docs' / 'figures'; FIG.mkdir(parents=True, exist_ok=True)


T = 1/fsw; Rt = rL + Rb; Vbus0 = Vref
Kpi = L*Wc_i/Vbus0            # current-loop PI: Kp sets crossover
Kii = Kpi*Wc_i/5              # PI zero at Wc/5

def run(t_end=0.12, windows=(0.025, 0.055, 0.115), Kp=None, Ki=None, ff=True):
    Kp = Kpi if Kp is None else Kp; Ki = Kii if Ki is None else Ki
    I = 0.0; xi = 0.0; rec = []; wf = []
    N = int(round(t_end/T))
    for n in range(N):
        t = n*T
        Iref = 0.0 if t < 0.002 else (10.0 if t < 0.03 else (20.0 if t < 0.09 else -10.0))
        Vbus = Vbus0 if t < 0.06 else 88.0           # 8 % bus sag from 60 ms
        dff = Vbat/(Vbus if ff else Vbus0)
        e = Iref - I                                   # sampled at mid-OFF = true cycle average (centre-aligned PWM)
        du = dff + Kp*e + Ki*(xi + e*T)
        d = min(max(du, 0.05), 0.95)
        if 0.05 < du < 0.95: xi += e*T
        rec.append((t, I, Iref, d, Vbus))
        segs = [((1-d)*T/2, 0.0), (d*T, Vbus), ((1-d)*T/2, 0.0)]
        inwin = any(w <= t < w + 5*T for w in windows); tt = t
        for dur, vs in segs:
            inf = (vs - Vbat)/Rt; a = Rt/L
            if inwin:
                for tau in np.linspace(0, dur, 10, endpoint=False):
                    wf.append((tt+tau, inf + (I-inf)*np.exp(-a*tau)))
            I = inf + (I-inf)*np.exp(-a*dur); tt += dur
    return np.array(rec), np.array(wf)

if __name__ == "__main__":
    # linear check of the sampled current loop (computation + PWM delay ~1.5 Ts)
    w = np.logspace(1, 4.5, 2000)
    G = (Vbus0/(1j*w*L + Rt))*np.exp(-1j*w*1.5*T); Lw = (Kpi + Kii/(1j*w))*G
    m = 20*np.log10(abs(Lw)); ph = np.degrees(np.unwrap(np.angle(Lw)))
    k = np.where(np.diff(np.sign(m)))[0][0]
    print(f"Kp={Kpi:.4f}  Ki={Kii:.3f}  crossover {w[k]:.0f} rad/s  PM {180+ph[k]:.1f} deg")

    rec, wf = run(); t, I, Iref, d, Vb = rec.T
    s1 = (t >= 0.002) & (t < 0.03)
    tr_lo = t[s1][np.argmax(I[s1] >= 1.0)]; tr_hi = t[s1][np.argmax(I[s1] >= 9.0)]
    print(f"0->10 A step: 10-90% rise {(tr_hi-tr_lo)*1e3:.2f} ms, overshoot {max(I[s1].max()-10,0):.2f} A")
    s2 = (t >= 0.03) & (t < 0.06)
    print(f"10->20 A step: overshoot {max(I[s2].max()-20,0):.2f} A, 2% settle {(t[s2][np.where(abs(I[s2]-20)>0.4)[0][-1]]-0.03)*1e3:.2f} ms")
    s3 = (t >= 0.06) & (t < 0.09)
    print(f"Bus sag 96->88 V at 60 ms: worst current error {abs(I[s3]-20).max():.2f} A, duty {d[(t>0.055)&(t<0.06)].mean():.3f} -> {d[(t>0.085)&(t<0.09)].mean():.3f}")
    s4 = (t >= 0.09)
    print(f"20 -> -10 A (charge -> discharge into bus): final I {I[-1]:.2f} A, duty {d[-50:].mean():.3f}, overshoot {max(-10-I[s4].min(),0):.2f} A")
    for w0 in (0.025, 0.055, 0.115):
        m_ = (wf[:,0] >= w0) & (wf[:,0] < w0+5*T)
        print(f"ripple @ {w0*1e3:.0f} ms: {wf[m_,1].max()-wf[m_,1].min():.2f} A pk-pk, mean {wf[m_,1].mean():.2f} A")
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    ax[0,0].plot(t*1e3, Iref, 'k--', lw=1, label="reference"); ax[0,0].plot(t*1e3, I, label="battery current (sampled)")
    ax[0,0].set(title="Current control: charge (+) to discharge (-)", xlabel="ms", ylabel="I (A)"); ax[0,0].legend()
    ax[0,1].plot(t*1e3, d); ax[0,1].set(title="Duty command", xlabel="ms", ylabel="d")
    ax[1,0].semilogx(w, m); ax[1,0].axhline(0, c='k', lw=.5); ax[1,0].set(title="Current-loop gain (with 1.5 Ts delay)", xlabel="rad/s", ylabel="dB")
    mm = (wf[:,0] >= 0.055) & (wf[:,0] < 0.055+5*T)
    ax[1,1].plot((wf[mm,0]-0.055)*1e6, wf[mm,1]); ax[1,1].set(title="Inductor current ripple at 20 A charge", xlabel="us", ylabel="I (A)")
    plt.tight_layout(); plt.savefig(FIG/"buck_overview.png", dpi=130)
