#!/usr/bin/env python3
"""
Pitch-axis closed-loop sim of the patched control path, shown in BOTH regimes:

  LEFT  - current airframe: pusher thrust line far above CG -> disturbance moment
          EXCEEDS pitch authority. STOCK crashes (matches 00000170.BIN). FF helps
          but also saturates -> still crashes. => software alone cannot fix it.

  RIGHT - after mechanical fix (EDF thrust line near CG): disturbance is WITHIN
          authority. STOCK holds but with a persistent nose-down error (needs
          constant back-stick). FF cancels it -> level, hands-off. => the win.
"""
import numpy as np, matplotlib.pyplot as plt

ANG_P=3.2; RAT_P=0.08; RAT_I=0.08; RAT_D=0.0036
RATE_MAX=np.radians(75.0); dt=1/400.0; T=9.0; n=int(T/dt)
I_yy=0.012; M_FULL=0.45; I_MAX=0.50; CRASH=np.radians(100.0)

def pusher(t):
    if t<1.5: return 0.0
    if t<5.0: return (t-1.5)/3.5
    return 1.0

def run(C_D, ff):
    th=0.0;q=0.0;integ=0.0;qp=0.0;crashed=None
    TT=[];TH=[];Q=[];QD=[];CMD=[];PU=[]
    for k in range(n):
        t=k*dt; p=pusher(t); p2=p*p; tau_d=-C_D*p2
        if crashed is None:
            err=0.0-th
            rdes=np.clip(ANG_P*err,-RATE_MAX,RATE_MAX)
            er=rdes-q
            base=RAT_P*er+integ+RAT_D*(-(q-qp)/dt)
            if abs(base)<1.0:
                integ=np.clip(integ+RAT_I*er*dt,-I_MAX,I_MAX)
            cmd=RAT_P*er+integ+RAT_D*(-(q-qp)/dt)+ff*p2
            cs=np.clip(cmd,-1,1); tau_c=M_FULL*cs
            qp=q; q+=(tau_c+tau_d)/I_yy*dt; th+=q*dt
            if abs(th)>CRASH:
                crashed=t; th=np.sign(th)*CRASH; q=0.0
        else:
            cmd=0; rdes=0
        TT.append(t);TH.append(np.degrees(th));Q.append(np.degrees(q))
        QD.append(np.degrees(rdes));CMD.append(np.clip(cmd,-1.5,1.5));PU.append(p)
    return dict(t=np.array(TT),th=np.array(TH),q=np.array(Q),qd=np.array(QD),
               cmd=np.array(CMD),pu=np.array(PU),crash=crashed)

CD_BIG=0.40          # MODERATE offset: stock cannot hold (integrator-limited), FF can
CD_SMALL=0.70        # LARGE offset: exceeds even FF authority
FF_BIG=CD_BIG/M_FULL       # 0.89 cancels exactly, no saturation
FF_SMALL=1.33               # best-effort, will saturate

A_stock=run(CD_BIG,0.0); A_ff=run(CD_BIG,FF_BIG)
B_stock=run(CD_SMALL,0.0); B_ff=run(CD_SMALL,FF_SMALL)

def rep(name,r):
    c=f"CRASHED at {r['crash']:.1f}s" if r['crash'] else f"stable, final pitch {r['th'][-1]:+.1f} deg"
    print(f"  {name:28s}: max|pitch|={np.abs(r['th']).max():5.1f} deg  worst rate={r['q'].min():+6.1f} deg/s  -> {c}")
print("LEFT  - MODERATE offset (within FF authority):")
rep("STOCK (ATC_PUSH_EN=0)",A_stock); rep("FF ON",A_ff)
print("RIGHT - LARGE offset (exceeds FF authority):")
rep("STOCK (ATC_PUSH_EN=0)",B_stock); rep("FF ON (calibrated)",B_ff)

fig,ax=plt.subplots(2,2,figsize=(14,10))
fig.patch.set_facecolor('#0f1419')
for a in ax.flat: a.set_facecolor('#11161c');a.tick_params(colors='#ccc');a.grid(alpha=0.2)

def pitch_panel(a,stock,ff,title,sub):
    a.plot(stock['t'],stock['th'],color='#ff4466',lw=2.2,label='STOCK (ATC_PUSH_EN=0)')
    a.plot(ff['t'],ff['th'],color='#33dd88',lw=2.2,label='FF ON')
    a.axhline(-30,color='#ffaa00',ls='--',lw=1,alpha=.7)
    a.text(0.1,-27,'-30 deg crash-check',color='#ffaa00',fontsize=8)
    if stock['crash']: a.scatter([stock['crash']],[ -100],color='#ff4466',marker='x',s=90,zorder=5)
    if ff['crash']:    a.scatter([ff['crash']],[-100],color='#33dd88',marker='x',s=90,zorder=5)
    a.set_title(title,color='#fff',fontsize=12,fontweight='bold')
    a.text(0.5,0.04,sub,transform=a.transAxes,ha='center',color='#9fb',fontsize=9)
    a.set_ylabel('Pitch angle (deg)',color='#ddd'); a.set_ylim(-110,40)
    a.legend(facecolor='#11161c',labelcolor='#ddd',loc='upper right',fontsize=9)

def cmd_panel(a,stock,ff):
    a.plot(stock['t'],stock['cmd'],color='#ff4466',lw=1.5,label='motor cmd STOCK')
    a.plot(ff['t'],ff['cmd'],color='#33dd88',lw=1.5,label='motor cmd FF')
    a.plot(stock['t'],stock['pu'],color='#66aaff',lw=1.4,ls=':',label='pusher cmd')
    a.axhline(1,color='#888',ls='--',lw=.7);a.axhline(-1,color='#888',ls='--',lw=.7)
    a.set_ylabel('normalized'); a.set_xlabel('time (s)',color='#ddd'); a.set_ylim(-1.6,1.6)
    a.legend(facecolor='#11161c',labelcolor='#ddd',loc='lower left',fontsize=8)

pitch_panel(ax[0,0],A_stock,A_ff,'MODERATE offset  (thrust line moderately above CG)',
            'STOCK crashes; the FF patch holds it level. <-- the win')
pitch_panel(ax[0,1],B_stock,B_ff,'LARGE offset  (thrust line far above CG)',
            'too far off: even FF saturates -> mechanical fix required')
cmd_panel(ax[1,0],A_stock,A_ff); cmd_panel(ax[1,1],B_stock,B_ff)
fig.suptitle('Pusher feed-forward patch — SITL-style control-law test (pitch axis)',
             color='#fff',fontsize=14,fontweight='bold')
plt.tight_layout(rect=[0,0,1,0.97])
plt.savefig('/tmp/sim_pusher_ff.png',dpi=140,facecolor='#0f1419')
print("\nsaved /tmp/sim_pusher_ff.png")
