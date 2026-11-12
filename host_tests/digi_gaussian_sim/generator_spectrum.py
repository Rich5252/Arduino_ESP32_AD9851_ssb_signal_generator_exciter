import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
FS=20000
def psd(fn):
    f=np.fromfile(fn,dtype=np.float32).astype(np.float64)
    z=np.exp(1j*2*np.pi*np.cumsum(f)/FS); z=z[FS//2:-FS//2]
    nper=2**15; w=np.blackman(nper); acc=0
    for s in range(0,len(z)-nper+1,nper//4): acc=acc+np.abs(np.fft.fft(z[s:s+nper]*w))**2
    p=np.fft.fftshift(acc); fr=np.fft.fftshift(np.fft.fftfreq(nper,1/FS)); p=p/p.sum()/(FS/nper)
    return fr,10*np.log10(p+1e-30)
def lv(fr,p,lo,hi,o):
    df=fr[1]-fr[0]; w=int(2/df)
    i1=np.argmin(abs(fr-(hi+o))); i2=np.argmin(abs(fr-(lo-o)))
    return 10*np.log10((np.mean(10**(p[i1-w:i1+w]/10))+np.mean(10**(p[i2-w:i2+w]/10)))/2)
rows=[('ft8_gfsk.f32','FT8 firmware generator, GFSK BT=2',1000,1043.75),('ft8_nano.f32','FT8 firmware generator, Nano curve',500,543.75),
      ('ft4_gfsk.f32','FT4 firmware generator, GFSK BT=1',1000,1062.5)]
res={}
print("%-40s"%"dBc/Hz beyond band edge"+"".join("%8s"%("+%d"%o) for o in (25,50,100,200)))
for fn,lab,lo,hi in rows:
    fr,p=psd(fn); res[fn]=(fr,p,lo,hi,lab)
    print("%-40s"%lab+"".join("%8.1f"%lv(fr,p,lo,hi,o) for o in (25,50,100,200)))
fig,axs=plt.subplots(2,1,figsize=(11,8.5),dpi=150,facecolor='#fcfcfb')
C=['#2a78d6','#eb6834']
ax=axs[0]
for (fn,c) in (('ft8_gfsk.f32',C[0]),('ft8_nano.f32',C[1])):
    fr,p,lo,hi,lab=res[fn]
    k=max(1,int(2/(fr[1]-fr[0]))); ps=10*np.log10(np.convolve(10**(p/10),np.ones(k)/k,mode='same'))
    ax.plot(fr-lo,ps,lw=1.6,color=c,label=lab)
ax.set_xlim(-200,250); ax.axvspan(0,43.75,color='#e8e7e2',zorder=0)
ax.set_title('FT8 - spectrum of the firmware generator output (host build, 20 kHz ticks)',loc='left',fontsize=12)
ax=axs[1]
fr,p,lo,hi,lab=res['ft4_gfsk.f32']; k=max(1,int(2/(fr[1]-fr[0]))); ps=10*np.log10(np.convolve(10**(p/10),np.ones(k)/k,mode='same'))
ax.plot(fr-lo,ps,lw=1.6,color=C[0],label=lab); ax.set_xlim(-200,262.5); ax.axvspan(0,62.5,color='#e8e7e2',zorder=0)
ax.set_title('FT4 - spectrum of the firmware generator output',loc='left',fontsize=12)
for ax in axs:
    ax.set_ylim(-140,-10); ax.set_facecolor('#fcfcfb'); ax.grid(color='#e3e2de',lw=0.6)
    for sp in ('top','right'): ax.spines[sp].set_visible(False)
    ax.set_ylabel('dBc/Hz',color='#52514e'); ax.legend(loc='upper right',frameon=False,fontsize=9); ax.tick_params(colors='#52514e')
axs[1].set_xlabel('Hz from tone 0',color='#52514e')
fig.tight_layout(); fig.savefig('digi_generator_spectrum.png',facecolor='#fcfcfb')
