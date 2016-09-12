/**
 * @file decimate.h
 * @brief  1D decimator using a gaussian filter followed by
 * downsample.
 *
 */

#ifndef __DECIMATE_H
#define __DECIMATE_H

class GaussianDecimator {
    int ksize,bsize; // kernel size, image buffer size
    double *k; // kernel
    double *blurbuf; // blur buffer (same size as image)

    // blur the image into the blur buffer
    void blur(double *f){
        for(int i=0;i<bsize;i++){
            double r = f[i]*k[0];
            for(int j=1;j<ksize;j++){
                // do both sides of the symmetric kernel, wrapping round
                int idx = i+j;
                idx%=bsize;
                r += f[idx]*k[j];
                idx = i-j;
                if(idx<0)idx+=bsize;
                r += f[idx]*k[j];
            }
            blurbuf[i]=r;
        }
    }
public:
    /// takes the size of the kernel, the sigma for the kernel,
    /// and the size of the linear image to be blurred.
    
    GaussianDecimator(int size, double sigma,int bufsize){
        if(!(size&1)){
            fprintf(stderr,"Kernel size must be odd\n");
            exit(1);
        }
        bsize = bufsize;
        blurbuf = new double[bsize];
        k = new double[size];
        
        double total=0;
        for(int i=0;i<size;i++){
            k[i] = exp(-0.5*(pow(((double)i)/sigma,2.0)))/
                  (2.0*M_PI*sigma*sigma);
            total+=k[i] * (i?2.0:1.0);
        }        
        for(int i=0;i<size;i++){
            k[i] /= total;
        }
        ksize=size;
    }
    ~GaussianDecimator(){
        delete [] k;
    }
    
    // source size is that passed into ctor
    void decimate(double *dest, int destct,double *src){
        blur(src);
        // and then downsample
        double step = (double)bsize/(double)destct;
        double pos = 0;
        for(int i=0;i<destct;i++){
            int i1 = (int)floor(pos);
            int i2 = ((int)ceil(pos))%bsize;
            double v1 = blurbuf[i1];
            double v2 = blurbuf[i2];
            double t = pos-floor(pos);
            dest[i] = v1*(1.0-t) + v2*t;
            pos += step;
        }
    }
    
    int getBufSize(){
        return bsize;
    }
};



#endif /* __DECIMATE_H */
