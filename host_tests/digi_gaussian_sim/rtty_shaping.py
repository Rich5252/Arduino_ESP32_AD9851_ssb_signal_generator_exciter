import numpy as np
from scipy.special import erf
FS=8000; Tb=1/45.45; NB=6000
rng=np.random.default_rng(7); bits=rng.integers(0,2,NB)
edges=np.arange(NB+1)*Tb; N=int(NB*Tb*FS); t=np.arange(N)/FS
lvl=np.where(bits==1,85.0,-85.0)
def freq(bt):
    idx=np.minimum((t/Tb).astype(int),NB-1); f=lvl[idx].copy()
    if bt is None: return f
    s=np.sqrt(np.log(2))/(2*np.pi*bt)*Tb; f=np.full(N,lvl[0])
    for k in range(1,NB):
        d=lvl[k]-lvl[k-1]
        if d==0: continue
        a=max(0,int((k*Tb-5*s)*FS)); b=min(N,int((k*Tb+5*s)*FS))
        f[a:b]+=d*0.5*(1+erf((t[a:b]-k*Tb)/(s*np.sqrt(2)))); f[b:]+=d
    return f
def spectrum(f):
    z=np.exp(1j*2*np.pi*np.cumsum(f)/FS); nper=2**15; w=np.blackman(nper); acc=0
    for s in range(0,len(z)-nper+1,nper//4): acc=acc+abs(np.fft.fft(z[s:s+nper]*w))**2
    p=np.fft.fftshift(acc); fr=np.fft.fftshift(np.fft.fftfreq(nper,1/FS)); p=p/p.sum()/(FS/nper)
    return fr,10*np.log10(p+1e-30)
def lev(fr,p,off):
    w=int(5/(fr[1]-fr[0])); i=np.argmin(abs(fr-(85+off))); j=np.argmin(abs(fr+85+off))
    return 10*np.log10((np.mean(10**(p[i-w:i+w]/10))+np.mean(10**(p[j-w:j+w]/10)))/2)
def ber(f,ebn0_db,trials=1):
    # non-coherent FSK, integrate-and-dump correlators at mark/space over each bit
    z=np.exp(1j*2*np.pi*np.cumsum(f)/FS); n=int(Tb*FS)
    Eb=n  # |z|^2 = 1 per sample
    N0=Eb/10**(ebn0_db/10); errs=0; tot=0
    rr=np.random.default_rng(11)
    noise=(rr.normal(size=N)+1j*rr.normal(size=N))*np.sqrt(N0/2)
    y=z+noise
    for k in range(2,NB-2):
        a=int(round(k*Tb*FS)); seg=y[a:a+n]; tt=t[a:a+n]
        m=abs(np.sum(seg*np.exp(-2j*np.pi*85*tt))); s_=abs(np.sum(seg*np.exp(2j*np.pi*85*tt)))
        errs+= ((m>s_)!=(bits[k]==1)); tot+=1
    return errs/tot
print("%-22s"%"keying"+"".join("%9s"%("+%dHz"%o) for o in (25,50,100,200,400))+"   BER@9dB  BER@11dB")
for lab,bt in (('hard (Nano)',None),('Gaussian BT 2.0',2.0),('Gaussian BT 1.0',1.0),('Gaussian BT 0.7',0.7),('Gaussian BT 0.5',0.5)):
    f=freq(bt); fr,p=spectrum(f)
    print("%-22s"%lab+"".join("%9.1f"%lev(fr,p,o) for o in (25,50,100,200,400))+"   %.4f   %.4f"%(ber(f,9),ber(f,11)))
    np.save('rtty_%s.npy'%('hard' if bt is None else str(bt)),np.vstack([fr,p]))
print("theory non-coherent FSK: 9 dB %.4f, 11 dB %.4f"%(0.5*np.exp(-10**0.9/2),0.5*np.exp(-10**1.1/2)))
for bt in (2.0,1.0,0.7,0.5):
    s=np.sqrt(np.log(2))/(2*np.pi*bt)*Tb; print("BT %.1f: sigma %.2f ms, 10-90%% edge %.1f ms (bit 22.0 ms)"%(bt,s*1e3,2.563*s*1e3))
