import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
C={'ref':'#2a78d6','nano':'#eb6834','str':'#1baf7a'}
fig,axs=plt.subplots(2,1,figsize=(11,9),dpi=150,facecolor='#fcfcfb')
for ax,mode,bt,steps,bw in ((axs[0],'FT8',2,256,43.75),(axs[1],'FT4',1,96,62.5)):
    d=np.load(mode+'.npy',allow_pickle=True).item()
    for key,lab,c in (('WSJT-X GFSK BT=%g, 50 us tick'%bt,'WSJT-X GFSK pulse (BT=%g), 50 µs steps'%bt,C['ref']),
                      ('Nano as written (%d x 64 us)'%steps,'Nano as written (%d × 64 µs); same curve at 50 µs steps'%steps,C['nano']),
                      ('Nano LUT stretched x5, 50 us','Nano lookup curve stretched ×5, 50 µs steps',C['str'])):
        fr,p=d[key]; df=fr[1]-fr[0]; k=max(1,int(2/df))
        ps=10*np.log10(np.convolve(10**(p/10),np.ones(k)/k,mode='same'))
        ax.plot(fr,ps,lw=1.6,color=c,label=lab)
    ax.set_facecolor('#fcfcfb'); ax.set_xlim(-200,bw+200); ax.set_ylim(-140,-10)
    ax.axvspan(0,bw,color='#e8e7e2',zorder=0)
    ax.grid(color='#e3e2de',lw=0.6); [s.set_visible(False) for s in ('top','right')for s in [ax.spines[s]]]
    ax.set_title('%s — spectrum around the tones (grey band = the %d tones)'%(mode,8 if mode=='FT8' else 4),loc='left',fontsize=12,color='#0b0b0b')
    ax.set_ylabel('dBc/Hz',color='#52514e'); ax.tick_params(colors='#52514e')
    ax.legend(loc='upper right',fontsize=9,frameon=False)
axs[1].set_xlabel('Hz from tone 0',color='#52514e')
fig.suptitle('Simulated FT8/FT4 spread: step time (64 vs 50 µs) changes nothing; transition shape changes a lot',x=0.06,ha='left',fontsize=13,color='#0b0b0b')
fig.tight_layout(rect=(0,0,1,0.96)); fig.savefig('digi_gaussian_spread.png',facecolor='#fcfcfb')
