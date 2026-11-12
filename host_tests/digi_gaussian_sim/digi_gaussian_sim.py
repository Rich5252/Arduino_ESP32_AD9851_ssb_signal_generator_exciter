import numpy as np, sys
from math import pi, sqrt, log
from scipy.special import erf as E
FS=1_000_000; FTWRES=180e6/2**32
LUT=np.array([0,518,705,948,1264,1667,2177,2815,3602,4565,5726,7110,8741,10638,12817,15288,18052,21102,24421,27978,31732,35630,39604,43582,47477,51203,54668,57782,60461,62631,64228,65206,65535])
def run(mode):
    if mode=='FT8': T,NS,M,sp,bt,steps=0.160,79,8,6.25,2.0,256
    else:           T,NS,M,sp,bt,steps=0.048,105*3,4,20.8333,1.0,96   # FT4: 3 messages back to back for a long enough record
    rng=np.random.default_rng(2); sym=rng.integers(0,M,NS)
    N=int(NS*T*FS); t=np.arange(N)/FS; base=sym*sp
    def nano(step_us=64, grid=None, stretch=1.0, jitter_ms=0):
        f=np.repeat(base,int(round(T*FS)))[:N].astype(float); f=np.pad(f,(0,N-len(f)),mode='edge')
        tl=steps*step_us*1e-6*stretch
        f0=np.arange(steps); vals=[]
        fct=steps//32
        for g in range(1,33):
            inc=(LUT[g]-LUT[g-1])//fct
            for h in range(1,fct+1): vals.append((LUT[g-1]+inc*h)/65536)
        vals=np.array(vals); jr=np.random.default_rng(5)
        for i in range(1,NS):
            tb=i*T+jr.uniform(0,jitter_ms*1e-3); t0=tb-tl/2
            a=int(round(t0*FS)); b=int(round((t0+tl)*FS))
            k=np.minimum(((t[a:b]-t0)/(tl/steps)).astype(int),steps-1)
            f[a:b]=base[i-1]+(base[i]-base[i-1])*vals[k]
        if grid: idx=(np.arange(N)//grid)*grid; f=f[idx]
        return np.round(f/FTWRES)*FTWRES
    def gfsk(grid=None, jitter_ms=0):
        c=pi*sqrt(2/log(2)); f=np.zeros(N); ts=t/T; jr=np.random.default_rng(5)
        for i in range(NS):
            sh=jr.uniform(0,jitter_ms*1e-3)/T
            lo=max(0,int((i-1.5)*T*FS)); hi=min(N,int((i+2.5)*T*FS)); x=ts[lo:hi]-i-0.5-sh
            f[lo:hi]+=base[i]*0.5*(E(c*bt*(x+0.5))-E(c*bt*(x-0.5)))
        h=int(T*FS); f[:h]=base[0]; f[-h:]=base[-1]
        if grid: idx=(np.arange(N)//grid)*grid; f=f[idx]
        return np.round(f/FTWRES)*FTWRES
    V={'WSJT-X GFSK BT=%g, 50 us tick'%bt: gfsk(50),
       'GFSK, 50 us tick, 1 ms symbol jitter': gfsk(50,1),
       'Nano as written (%d x 64 us)'%steps: nano(),
       'Nano LUT, 50 us tick': nano(grid=50),
       'Nano LUT, 1 ms symbol jitter': nano(grid=50,jitter_ms=1),
       'Nano LUT stretched x3, 50 us': nano(grid=50,stretch=3),
       'Nano LUT stretched x5, 50 us': nano(grid=50,stretch=5)}
    out={}
    for k,f in V.items():
        z=np.exp(1j*2*np.pi*np.cumsum(f)/FS).astype(np.complex64); a=int(0.5*FS); z=z[a:N-a]
        nper=2**20 if mode=='FT4' else 2**21; w=np.blackman(nper).astype(np.float32); acc=0;n=0
        for s in range(0,len(z)-nper+1,nper//2):
            acc=acc+np.abs(np.fft.fft(z[s:s+nper]*w))**2; n+=1
        p=np.fft.fftshift(acc/n); fr=np.fft.fftshift(np.fft.fftfreq(nper,1/FS)); p=p/p.sum()/(FS/nper)
        sel=(fr>-1500)&(fr<1600); out[k]=(fr[sel],10*np.log10(p[sel]+1e-30))
        bw=(M-1)*sp; df=fr[1]-fr[0]
        def lv(o):
            i1=np.argmin(abs(fr-(bw+o))); i2=np.argmin(abs(fr+o)); w2=int(2/df)
            return 10*np.log10(((10**(10*np.log10(p[i1-w2:i1+w2]+1e-30)/10)).mean()+(10**(10*np.log10(p[i2-w2:i2+w2]+1e-30)/10)).mean())/2)
        print("%-40s"%(mode+' '+k)+"".join("%8.1f"%lv(o) for o in (10,25,50,100,200,500)))
    np.save(mode+'.npy',out,allow_pickle=True)
print("%-40s"%"dBc/Hz beyond band edge"+"".join("%8s"%("+%d"%o) for o in (10,25,50,100,200,500)))
run('FT8'); run('FT4')
