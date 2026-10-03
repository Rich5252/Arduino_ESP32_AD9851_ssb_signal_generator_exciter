/* Host-side check of the folded Hilbert FIR used in ssb_dsp.c (2026-10-03).
 * Build:  gcc -O2 -o test_hilbert_fold test_hilbert_fold.c -lm && ./test_hilbert_fold
 * 1) folded loop vs the original full-convolution (two-range) loop, every head position, random data
 * 2) end-to-end image (opposite sideband) level for a tone through I + jQ, to compare with sim_eer/sim_hilbert_image.py
 * The generate/FIR code below is copied from ssb_dsp.c; keep in sync if that changes. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static void generate_hilbert_coeffs(float *coeffs, int num_taps){
    int center=(num_taps-1)/2;
    for(int n=0;n<num_taps;n++){ int k=n-center; float h;
        if(k==0||(k%2)==0) h=0.0f; else h=2.0f/(M_PI*(float)k);
        float w=0.54f-0.46f*cosf(2.0f*M_PI*(float)n/(float)(num_taps-1)); coeffs[n]=h*w; } }
static float fir_ref(const float*c,const float*dl,int N,int head){
    float Q=0; int n=0; for(;n<=head;n++) Q+=c[n]*dl[head-n]; for(;n<N;n++) Q+=c[n]*dl[head-n+N]; return Q; }
static float fir_fold(const float*hh,int half_n,const float*dl,int N,int center,int head){
    int base=head-center; if(base<0) base+=N; float Q=0;
    for(int j=0;j<half_n;j++){ int k=2*j+1; int ia=base-k; if(ia<0) ia+=N; int ib=base+k; if(ib>=N) ib-=N; Q+=hh[j]*(dl[ia]-dl[ib]); }
    return Q; }
int main(void){
    int taps_list[]={3,5,7,33,65,129}; int fails=0;
    for(int t=0;t<6;t++){ int N=taps_list[t], center=(N-1)/2, half_n=(center+1)/2;
        float*c=calloc(N,4),*hh=calloc(half_n,4),*dl=calloc(N,4); generate_hilbert_coeffs(c,N);
        for(int j=0;j<half_n;j++) hh[j]=c[center+(2*j+1)];
        for(int k=1;k<=center;k++){ float a=c[center+k],b=c[center-k]; if(fabsf(a+b)>1e-7f){printf("antisymmetry FAIL N=%d k=%d\n",N,k);fails++;} }
        for(int n=0;n<N;n++) if(((n-center)%2==0) && c[n]!=0.0f){printf("even-tap nonzero N=%d n=%d\n",N,n);fails++;}
        srand(1); for(int i=0;i<N;i++) dl[i]=(float)rand()/RAND_MAX*2-1;
        double maxd=0; for(int head=0;head<N;head++){ float a=fir_ref(c,dl,N,head),b=fir_fold(hh,half_n,dl,N,center,head); double d=fabs(a-b); if(d>maxd)maxd=d; }
        printf("taps %3d: max |ref - folded| over all head positions = %.3g  (MACs: %d -> %d)\n",N,maxd,N,half_n); if(maxd>1e-5) fails++;
        free(c);free(hh);free(dl); }
    /* end-to-end image level */
    printf("\nimage (opposite sideband) of a tone through z = I + jQ, dB re wanted sideband:\n  Fs  taps  f=150   250   350  400\n");
    double fsl[2]={16000,20000}; int tl[3]={65,81,129};
    for(int a=0;a<2;a++) for(int b=0;b<3;b++){ double fs=fsl[a]; if(a==0&&b==1) continue; int N=tl[b],center=(N-1)/2,half_n=(center+1)/2;
        float*c=calloc(N,4),*hh=calloc(half_n,4),*dl=calloc(N,4); generate_hilbert_coeffs(c,N); for(int j=0;j<half_n;j++) hh[j]=c[center+(2*j+1)];
        printf("%5.0f %4d  ",fs,N);
        double fl[4]={150,250,350,400};
        for(int fi=0;fi<4;fi++){ double f=fl[fi]; int head=0; memset(dl,0,N*4); int M=(int)(fs/f*20); /* 20 cycles-ish; use exact bins below */
            int L=40000; double complex_re_p=0,complex_im_p=0,re_n=0,im_n=0; double w=2*M_PI*f/fs;
            for(int n=0;n<L+N;n++){ head++; if(head>=N) head=0; dl[head]=(float)cos(w*n);
                int ii=head-center; if(ii<0) ii+=N; float I=dl[ii]; float Q=fir_fold(hh,half_n,dl,N,center,head);
                if(n>=N){ int m=n-N; double ph=w*m; /* project z=I+jQ onto e^{+jwm} and e^{-jwm}, Hann window */
                    double win=0.5-0.5*cos(2*M_PI*m/L);
                    complex_re_p+=win*(I*cos(ph)+Q*sin(ph)); complex_im_p+=win*(Q*cos(ph)-I*sin(ph));
                    re_n+=win*(I*cos(ph)-Q*sin(ph)); im_n+=win*(Q*cos(ph)+I*sin(ph)); } }
            double P=sqrt(complex_re_p*complex_re_p+complex_im_p*complex_im_p), Nn=sqrt(re_n*re_n+im_n*im_n);
            printf("%6.1f ",20*log10(Nn/P+1e-12)); }
        printf("\n"); free(c);free(hh);free(dl); }
    printf("\n%s\n",fails?"FAILURES":"all checks passed"); return fails?1:0; }
