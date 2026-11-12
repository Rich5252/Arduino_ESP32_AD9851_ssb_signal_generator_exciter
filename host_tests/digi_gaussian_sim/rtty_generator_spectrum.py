import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
FS=20000
def psd(fn):
    f=np.fromfile(fn,dtype=np.float32).astype(np.float64)
    z=np.exp(1j*2*np.pi*np.cumsum(f)/FS); nper=2**16; w=np.blackman(nper); acc=0
    for s in range(0,len(z)-nper+1,nper//4): acc=acc+np.abs(np.fft.fft(z[s:s+nper]*w))**2
    p=np.fft.fftshift(acc); fr=np.fft.fftshift(np.fft.fftfreq(nper,1/FS)); p=p/p.sum()/(FS/nper)
    return fr,10*np.log10(p+1e-30)
def lv(fr,p,o):
    w=int(5/(fr[1]-fr[0])); i=np.argmin(abs(fr-(85+o))); j=np.argmin(abs(fr+85+o))
    return 10*np.log10((np.mean(10**(p[i-w:i+w]/10))+np.mean(10**(p[j-w:j+w]/10)))/2)
rows=[('rtty_hard.f32','Hard keyed (Nano, DIGI_BT_RTTY 0)','#eb6834'),('rtty_bt2.f32','Gaussian BT 2.0 (3.7 ms edges)','#2a78d6'),('rtty_bt1.f32','Gaussian BT 1.0 (7.5 ms edges)','#1baf7a')]
print("%-38s"%"dBc/Hz beyond mark/space"+"".join("%8s"%("+%d"%o) for o in (50,100,200,400,800)))
fig,ax=plt.subplots(figsize=(11,5.2),dpi=150,facecolor='#fcfcfb')
for fn,lab,c in rows:
    fr,p=psd(fn); print("%-38s"%lab+"".join("%8.1f"%lv(fr,p,o) for o in (50,100,200,400,800)))
    k=max(1,int(4/(fr[1]-fr[0]))); ps=10*np.log10(np.convolve(10**(p/10),np.ones(k)/k,mode='same'))
    ax.plot(fr,ps,lw=1.5,color=c,label=lab)
ax.set_xlim(-700,700); ax.set_ylim(-150,-10); ax.axvspan(-85,85,color='#e8e7e2',zorder=0)
ax.set_facecolor('#fcfcfb'); ax.grid(color='#e3e2de',lw=0.6)
for sp in ('top','right'): ax.spines[sp].set_visible(False)
ax.set_title('RTTY 45.45 Bd / 170 Hz - spectrum of the firmware generator output (host build)',loc='left',fontsize=12)
ax.set_xlabel('Hz from the carrier (space -85, mark +85)',color='#52514e'); ax.set_ylabel('dBc/Hz',color='#52514e')
ax.legend(loc='upper right',frameon=False,fontsize=9); ax.tick_params(colors='#52514e')
fig.tight_layout(); fig.savefig('rtty_shaping_spectrum.png',facecolor='#fcfcfb')
