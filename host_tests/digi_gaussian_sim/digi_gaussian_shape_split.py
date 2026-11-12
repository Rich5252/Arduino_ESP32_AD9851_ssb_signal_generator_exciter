import numpy as np
from scipy.special import erf
FS=1_000_000
L=np.array([0,518,705,948,1264,1667,2177,2815,3602,4565,5726,7110,8741,10638,12817,15288,18052,21102,24421,27978,31732,35630,39604,43582,47477,51203,54668,57782,60461,62631,64228,65206,65535])/65535
def run(mode):
    if mode=='FT8': T,NS,M,sp,tl,sig_n,sig_w=0.160,79,8,6.25,256*64e-6,0.205*256*64e-6,10.60e-3
    else:           T,NS,M,sp,tl,sig_n,sig_w=0.048,315,4,20.8333,96*64e-6,0.205*96*64e-6,6.36e-3
    sym=np.random.default_rng(2).integers(0,M,NS); base=sym*sp
    N=int(NS*T*FS); t=np.arange(N)/FS
    def build(shape, win):     # shape(u) for u = time from boundary (s); applied within +/-win/2
        f=np.repeat(base,int(round(T*FS)))[:N].astype(float)
        for i in range(1,NS):
            tb=i*T; a=int(round((tb-win/2)*FS)); b=int(round((tb+win/2)*FS))
            f[a:b]=base[i-1]+(base[i]-base[i-1])*shape(t[a:b]-tb)
        return f
    lut=lambda u: np.interp((u+tl/2)/tl, np.arange(33)/32, L)
    E=lambda s,cut: (lambda u: np.clip((0.5*(1+erf(u/(s*np.sqrt(2))))-0.5)/(1-2*(0.5*(1-erf(cut/2/(s*np.sqrt(2))))))+0.5,0,1))
    V=[('Nano LUT as written (off-centre, skewed, end jumps)', build(lut,tl)),
       ('symmetric erf, Nano width, same window (end jumps)', build(lambda u:0.5*(1+erf(u/(sig_n*np.sqrt(2)))),tl)),
       ('symmetric erf, Nano width, no truncation', build(lambda u:0.5*(1+erf(u/(sig_n*np.sqrt(2)))),min(10*sig_n,T))),
       ('symmetric erf, WSJT-X width, no truncation', build(lambda u:0.5*(1+erf(u/(sig_w*np.sqrt(2)))),T))]
    for k,f in V:
        z=np.exp(1j*2*np.pi*np.cumsum(f)/FS).astype(np.complex64); z=z[int(.5*FS):N-int(.5*FS)]
        nper=2**20; w=np.blackman(nper).astype(np.float32); acc=0
        for s in range(0,len(z)-nper+1,nper//2): acc=acc+np.abs(np.fft.fft(z[s:s+nper]*w))**2
        p=np.fft.fftshift(acc); fr=np.fft.fftshift(np.fft.fftfreq(nper,1/FS)); p=p/p.sum()/(FS/nper)
        bw=(M-1)*sp; df=fr[1]-fr[0]; w2=int(2/df)
        def lv(o):
            i1=np.argmin(abs(fr-(bw+o))); i2=np.argmin(abs(fr+o))
            return 10*np.log10((p[i1-w2:i1+w2].mean()+p[i2-w2:i2+w2].mean())/2+1e-30)
        print("%-4s %-52s"%(mode,k)+"".join("%8.1f"%lv(o) for o in (25,50,100,200,500)))
print("%-57s"%"dBc/Hz beyond band edge"+"".join("%8s"%("+%d"%o) for o in (25,50,100,200,500)))
run('FT8'); run('FT4')
