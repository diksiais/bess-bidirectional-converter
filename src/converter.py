import numpy as np
from scipy.integrate import solve_ivp
from scipy import signal
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from params import *
from pathlib import Path
FIG = Path(__file__).resolve().parent.parent / 'docs' / 'figures'; FIG.mkdir(parents=True, exist_ok=True)


def plant_rhs(t, x, d, R):
    iL, vC = x
    return [(Vin-(1-d)*vC)/L, ((1-d)*iL - vC/R)/C]

# ---- Step 1: open-loop nonlinear step (duty 0.50 -> 0.55)
def open_loop():
    xs = [Vin/(1-D0)/(R0*(1-D0)), Vref]
    f = lambda t,x: plant_rhs(t,x, D0 if t<0.05 else 0.55, R0)
    s = solve_ivp(f,[0,0.4],xs,max_step=2e-4)
    return s

# ---- Step 2: linearised model  duty -> vC
def linear_model():
    Ie = Vref/(R0*(1-D0))
    A = np.array([[0,-(1-D0)/L],[(1-D0)/C,-1/(R0*C)]])
    B = np.array([[Vref/L],[-Ie/C]])
    Cm = np.array([[0,1.0]])
    sys = signal.StateSpace(A,B,Cm,[[0]])
    return signal.TransferFunction(sys), A

# ---- Step 3/4: closed-loop nonlinear sim with load step
def closed_loop(Kp=Kp, Ki=Ki, t_step=0.3, R1=9.6, T=1.0):
    dmin,dmax = 0.05,0.90
    def f(t,x):
        iL,vC,xi = x
        R = R0 if t<t_step else R1
        e = Vref - vC                 # see README: sign set so feedback is negative
        d = np.clip(D0 + Kp*e + Ki*xi, dmin, dmax)
        sat = (d>=dmax and e>0) or (d<=dmin and e<0)   # anti-windup
        return [*plant_rhs(t,[iL,vC],d,R), 0.0 if sat else e]
    x0 = [Vref/(R0*(1-D0)), Vref, 0.0]
    s = solve_ivp(f,[0,T],x0,max_step=1e-4,method="LSODA")
    d = np.clip(D0+Kp*(Vref-s.y[1])+Ki*s.y[2],dmin,dmax)
    return s,d

if __name__=="__main__":
    tf,A = linear_model()
    num,den = tf.num, tf.den
    print("poles:", np.roots(den)); print("zeros:", np.roots(num))
    print("DC gain:", num[-1]/den[-1], " |wn|:",abs(np.roots(den)[0]))
    w=np.logspace(1,5,800); w,mag,ph=signal.bode(tf,w)
    # loop gain L(s)=C(s)G(s), C=Kp+Ki/s
    _,Gc = signal.freqresp(tf,w); Lw=(Kp+Ki/(1j*w))*Gc
    # open-loop
    s=open_loop()
    # closed loop
    sc,d=closed_loop()
    fig,ax=plt.subplots(2,2,figsize=(12,8))
    ax[0,0].plot(s.t*1e3,s.y[1]); ax[0,0].set(title="1. Open-loop duty step 0.50->0.55",xlabel="ms",ylabel="Vbus (V)")
    ax[0,1].semilogx(w,mag); ax[0,1].set(title="2. Bode duty->Vbus (magnitude)",xlabel="rad/s",ylabel="dB")
    ax[1,0].semilogx(w,20*np.log10(abs(Lw))); ax[1,0].axhline(0,c='k',lw=.5); ax[1,0].set(title="3. PI loop gain",xlabel="rad/s",ylabel="dB")
    ax[1,1].plot(sc.t,sc.y[1]); ax[1,1].axvline(0.3,c='r',ls='--'); ax[1,1].set(title="4. Closed loop, load 14.4->9.6 ohm at 0.3 s",xlabel="s",ylabel="Vbus (V)")
    plt.tight_layout(); plt.savefig(FIG/"overview.png",dpi=130)
    # metrics
    v=sc.y[1]; t=sc.t
    i=t>0.3; dip=Vref-v[i].min()
    band=abs(v-Vref)<0.02*Vref; idx=np.where(~band & i)[0]; ts=t[idx[-1]]-0.3 if len(idx) else 0
    print(f"load-step dip {dip:.2f} V, 2% settling {ts*1e3:.0f} ms, duty range {d.min():.3f}-{d.max():.3f}, final v {v[-1]:.2f}")
    # crossover & phase margin
    k=np.where(np.diff(np.sign(20*np.log10(abs(Lw)))))[0]
    for j in k: print("crossover %.0f rad/s, PM %.1f deg"%(w[j],180+np.degrees(np.unwrap(np.angle(Lw)))[j]))
