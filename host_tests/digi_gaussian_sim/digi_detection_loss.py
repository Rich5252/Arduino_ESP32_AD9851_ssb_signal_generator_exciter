import numpy as np
from scipy.special import erf
FS=48000
L=np.array([0,518,705,948,1264,1667,2177,2815,3602,4565,5726,7110,8741,10638,12817,15288,18052,21102,24421,27978,31732,35630,39604,43582,47477,51203,54668,57782,60461,62631,64228,65206,65535])/65535
def run(T,M,sp,bt,steps,name):
    rng=np.random.default_rng(3); NS=4000; sym=rng.integers(0,M,NS)
    n=int(round(T*FS)); N=NS*n; t=np.arange(N)/FS
    base=np.repeat(sym*sp,n).astype(float)
    def gfsk():
        sig=np.sqrt(np.log(2))/(2*np.pi*bt)*T; f=np.full(N,sym[0]*sp)
        for k in range(1,NS): 
            a=max(0,k*n-int(5*sig*FS)); b=min(N,k*n+int(5*sig*FS))
            f[a:b]+=(sym[k]-sym[k-1])*sp*0.5*(1+erf((t[a:b]-k*T)/(sig*np.sqrt(2))))
            f[b:]+=(sym[k]-sym[k-1])*sp
        return f
    def nano():
        tl=steps*64e-6; f=base.copy()
        for k in range(1,NS):
            t0=k*T-tl/2; a=int(t0*FS); b=int((t0+tl)*FS)
            f[a:b]=sym[k-1]*sp+(sym[k]-sym[k-1])*sp*np.interp((t[a:b]-t0)/tl,np.arange(33)/32,L)
        return f
    out=[]
    for lab,f in (('ideal FSK',base),('Nano curve',nano()),('GFSK BT=%g'%bt,gfsk())):
        ph=2*np.pi*np.cumsum(f)/FS; z=np.exp(1j*ph)
        # per-symbol non-coherent tone power at the right tone vs symbol energy (as an FFT-bin detector)
        loss=[];leak=[]
        for k in range(2,NS-2):
            seg=z[k*n:(k+1)*n]; tt=t[k*n:(k+1)*n]
            p=[abs(np.sum(seg*np.exp(-2j*np.pi*m*sp*tt)))**2/n**2 for m in range(M)]
            loss.append(p[sym[k]]); leak.append(max(p[m] for m in range(M) if m!=sym[k]))
        out.append((lab,10*np.log10(np.mean(loss)),10*np.log10(np.mean(leak))))
    print(name)
    for lab,l,k in out: print("  %-12s wanted-tone power %6.2f dB   worst other tone %6.1f dB"%(lab,l,k))
run(0.160,8,6.25,2.0,256,'FT8')
run(0.048,4,12000/576,1.0,96,'FT4')
