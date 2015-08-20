#ifndef AN_FLUID_H
#define AN_FLUID_H

#include "an.lua.h"
#include <list>

struct Fluid {

  unsigned int bits, dim, dim2, dim3, dimwrap;
  unsigned int scale; // division of region dimension to fluid dimension (pow of 2)
  double  *pfield, *pfield0;    // pressure field
  double  *uxfield, *uxfield0;  // velocity field 
  double  *uyfield, *uyfield0;  // velocity field 
  double  *uzfield, *uzfield0;  // velocity field 
  
  double  *noisefield;      // a source of turbulence
  double noise;
  
  double viscosity, diffusion, decay;
  
  Fluid(  unsigned int dim, 
      double viscosity = 0.00001, 
      double diffusion = 0.001, 
      double decay = 0.99,
      unsigned int scale = 2) 
    : dim(dim), viscosity(viscosity), diffusion(diffusion), decay(decay), scale(scale)
  {
    dim2 = dim*dim;
    dim3 = dim*dim*dim;
    dimwrap = dim-1;
    
    noisefield = new double[dim3];
    noise = 0;
    
    pfield = new double[dim3];
    pfield0 = new double[dim3];
    
    uxfield = new double[dim3];
    uyfield = new double[dim3];
    uzfield = new double[dim3];
    uxfield0 = new double[dim3];
    uyfield0 = new double[dim3];
    uzfield0 = new double[dim3];
    
    reset();
  }
  
  ~Fluid() {
    delete[] pfield;
    delete[] pfield0;

    delete[] noisefield;
    
    delete[] uxfield;
    delete[] uyfield;
    delete[] uzfield;
    delete[] uxfield0;
    delete[] uyfield0;
    delete[] uzfield0;
  }
  
  inline unsigned int index(int x, int y, int z) {
    return ((x/scale) & dimwrap)*dim2 
       + ((y/scale) & dimwrap)*dim 
       + ((z/scale) & dimwrap);
  }
  inline unsigned int index(double x, double y, double z) {
    return index((int)floor(x), (int)floor(y), (int)floor(z));
  }
  
  inline void reset() {
    for (int i=0; i<dim3; i++) {
      pfield[i] = 0.;
      pfield0[i] = pfield[i];
      
      noisefield[i] = 0;
      
      uxfield[i] = srandom();
      uyfield[i] = srandom();
      uzfield[i] = srandom();
      uxfield0[i] = uxfield[i];
      uyfield0[i] = uyfield[i];
      uzfield0[i] = uzfield[i];
    }
  }
  
  inline void getFlow(vec & pos, vec & flow) {
    int i = index(pos.x, pos.y, pos.z);
    flow.x = uxfield[i];
    flow.y = uyfield[i];
    flow.z = uzfield[i];
  }
      
  inline void getFlowInterp(vec & pos, vec & flow) {
    double x = pos.x/(double)scale; 
    double y = pos.y/(double)scale; 
    double z = pos.z/(double)scale;
    
    // get the integer components, and normalized 0..1 interp factors, of x,y,z:
    int x0 = (int)x; double xbf = x-(double)x0; double xaf = 1.-xbf; 
    int y0 = (int)y; double ybf = y-(double)y0; double yaf = 1.-ybf;
    int z0 = (int)z; double zbf = z-(double)z0; double zaf = 1.-zbf;
    
    // find the 8 cube corners by getting the corners' array indices:
    int xa = (x0 & (dimwrap))*dim2; int xb = ((x0+1) & (dimwrap))*dim2;
    int ya = (y0 & (dimwrap))*dim;  int yb = ((y0+1) & (dimwrap))*dim;
    int za = (z0 & (dimwrap));    int zb = ((z0+1) & (dimwrap));
    
    int iaaa = xa + ya + za; double faaa = xaf * yaf * zaf;
    int ibaa = xb + ya + za; double fbaa = xbf * yaf * zaf;
    int iaba = xa + yb + za; double faba = xaf * ybf * zaf;
    int iaab = xa + ya + zb; double faab = xaf * yaf * zbf;
    int ibab = xb + ya + zb; double fbab = xbf * yaf * zbf;
    int iabb = xa + yb + zb; double fabb = xaf * ybf * zbf;
    int ibba = xb + yb + za; double fbba = xbf * ybf * zaf;
    int ibbb = xb + yb + zb; double fbbb = xbf * ybf * zbf;
    
    // do the interpolation:
    flow.x =(uxfield[iaaa] * faaa) +
        (uxfield[ibaa] * fbaa) + 
        (uxfield[iaba] * faba) + 
        (uxfield[iaab] * faab) +
        (uxfield[ibab] * fbab) + 
        (uxfield[iabb] * fabb) + 
        (uxfield[ibba] * fbba) + 
        (uxfield[ibbb] * fbbb);
        
    flow.y =(uyfield[iaaa] * faaa) +
        (uyfield[ibaa] * fbaa) + 
        (uyfield[iaba] * faba) + 
        (uyfield[iaab] * faab) +
        (uyfield[ibab] * fbab) + 
        (uyfield[iabb] * fabb) + 
        (uyfield[ibba] * fbba) + 
        (uyfield[ibbb] * fbbb);
        
    flow.z =(uzfield[iaaa] * faaa) +
        (uzfield[ibaa] * fbaa) + 
        (uzfield[iaba] * faba) + 
        (uzfield[iaab] * faab) +
        (uzfield[ibab] * fbab) + 
        (uzfield[iabb] * fabb) + 
        (uzfield[ibba] * fbba) + 
        (uzfield[ibbb] * fbbb);
  }
  
  inline void mixFlow(vec & pos, vec & vel, double mix) {
    int i = index(pos.x, pos.y, pos.z);
    uxfield[i] += mix * (2.*vel.x - uxfield0[i]);
    uyfield[i] += mix * (2.*vel.y - uyfield0[i]);
    uzfield[i] += mix * (2.*vel.z - uzfield0[i]);
  }
  
  inline void addNoise(vec & pos, double amt) {
    int i = index(pos.x, pos.y, pos.z);
    pfield[i] += amt;
  }
  
  inline double getNoise(vec & pos) {
    int i = index(pos.x, pos.y, pos.z);
    return pfield[i];
  }

  
  inline void step();
};


namespace stable {

  #define INDEX(x, y, z) ((z & (N-1)) + (y & (N-1))*N + (x & (N-1))*N*N)
  #define BEGIN_PER_CELL  for (int x=0 ; x<N ; x++ ) { \
              for (int y=0 ; y<N ; y++ ) { \
              for (int z=0 ; z<N ; z++ ) { \
                int i = INDEX(x, y, z);
  #define END_PER_CELL  }}}
  #define SWAP_PTR(x0,x) {double * tmp=x0;x0=x;x=tmp;}

  /*
    Stable diffusion solver
      for dimension N, diffuses field p (previous p0)
    
    This method avoids oscillation and divergence in diffusion, 
      granting a *stable* result instead:
      an iterative technique to solve a sparse linear system
      (the Gauss-Seidel relaxation method)  
  */
  inline void stable_solve(int N, double * p, double * p0, double diffusion, double divisor ) {
    static int iterations = 16; //20; 
    double div = 1.0/divisor;
    
    for (int n=0 ; n<iterations ; n++) {  
      BEGIN_PER_CELL  
        p[i] = (
              p0[i] + 
              diffusion*(
                p[INDEX(x-1, y, z)] + p[INDEX(x+1, y, z)] +
                p[INDEX(x, y-1, z)] + p[INDEX(x, y+1, z)] +
                p[INDEX(x, y, z-1)] + p[INDEX(x, y, z+1)]
              )
            ) * div;
      END_PER_CELL
    }
  }

  inline void diffuse(int N, double * p, double * p0, double diffusion)
  {
    double a = diffusion; // * (N * N * N);
    stable_solve(N, p, p0, a, (1+6*a) );
  }

  // (linear) interpolate a value at the given coords:
  inline double interp(int N, double * p0, double x, double y, double z) 
  {
    int dimwrap = N-1; // assumes N is a power of 2!!
    
    // get the integer components, and normalized 0..1 interp factors, of x,y,z:
    int x0 = (int)x; double xbf = x-(double)x0; double xaf = 1-xbf; 
    int y0 = (int)y; double ybf = y-(double)y0; double yaf = 1-ybf;
    int z0 = (int)z; double zbf = z-(double)z0; double zaf = 1-zbf;
    
    // find the 8 cube corners by getting the corners' array indices:
    int xa = (x0 & (dimwrap))*N*N;  int xb = ((x0+1) & (dimwrap))*N*N;
    int ya = (y0 & (dimwrap))*N;  int yb = ((y0+1) & (dimwrap))*N;
    int za = (z0 & (dimwrap));    int zb = ((z0+1) & (dimwrap));
    
    // do the interpolation:
    return  (p0[xa + ya + za] * xaf * yaf * zaf) +
        (p0[xb + ya + za] * xbf * yaf * zaf) + 
        (p0[xa + yb + za] * xaf * ybf * zaf) + 
        (p0[xa + ya + zb] * xaf * yaf * zbf) +
        (p0[xb + ya + zb] * xbf * yaf * zbf) + 
        (p0[xa + yb + zb] * xaf * ybf * zbf) + 
        (p0[xb + yb + za] * xbf * ybf * zaf) + 
        (p0[xb + yb + zb] * xbf * ybf * zbf);
  }

  /*
    Translate densities etc. (p, p0 = previous) through the vector field (vx, vy, vz) over dim N
      Does a linear back-trace of center point through vector field, 
      using linear interpolation to gather p0 at this source and place in new p
  */
  inline void advect(int N, double * p, double * p0, double * vx, double * vy, double * vz)
  {
    static double rate = 1.0;
    BEGIN_PER_CELL
      // back trace:
      double x0 = x - rate * vx[i]; 
      double y0 = y - rate * vy[i]; 
      double z0 = z - rate * vz[i]; 
      // trilinearinterp p0 at (x0, y0, z0):
      p[i] = interp(N, p0, x0, y0, z0);
    END_PER_CELL
  }

  /*
    Clever part of Jos Stam's work. 
      A velocity field can become divergent (have regions that are purely emanating or aggregating)
        violating the definition of an incompressible fluid
      But, since a velocity field can be seen as an incompressible velocity field + a gradient field,
        we can subtract a gradient field from our bad velocity field to get an incompressible one
      To calculate this gradient field and then subtract it, we use this function:
  */
  inline void project(int N, double * vx, double * vy, double * vz, double * g, double * g0 )
  {
    double h = 1.0/N;
    
    // g and g0 represent the velocity gradient
    BEGIN_PER_CELL
      // previous instantaneous magnitude of velocity gradient 
      //    = (sum of velocity gradients per axis)/2N:
      g0[i] = -0.5 * h * (
                  vx[INDEX(x+1, y, z)]-vx[INDEX(x-1, y, z)]+  // velocity gradients
                  vy[INDEX(x, y+1, z)]-vy[INDEX(x, y-1, z)]+  // velocity gradients
                  vz[INDEX(x, y, z+1)]-vz[INDEX(x, y, z-1)] // velocity gradients
                );
      // zero out the present velocity gradient:
      g[i] = 0;
    END_PER_CELL
    
    // reuse the Gauss-Seidel relaxation solver to safely diffuse the velocity gradients from g0 to g:
    stable_solve(N, g, g0, 1, 6);
    
    // now subtract this gradient from our current velocity field:
    BEGIN_PER_CELL
      vx[i] -= 0.5 * N * (g[INDEX(x+1, y, z)]-g[INDEX(x-1, y, z)]); // gradient calculated by neighbors
      vy[i] -= 0.5 * N * (g[INDEX(x, y+1, z)]-g[INDEX(x, y-1, z)]);
      vz[i] -= 0.5 * N * (g[INDEX(x, y, z+1)]-g[INDEX(x, y, z-1)]);
    END_PER_CELL
  }

  void dens_step (int N, double * p, double * p0, double * vx, double * vy, double * vz, double diffusion )
  {
    SWAP_PTR( p0, p ); 
    diffuse(N, p, p0, diffusion);
    SWAP_PTR( p0, p ); 
    advect(N, p, p0, vx, vy, vz);
  }

  void vel_step (int N, double * vx, double * vy, double * vz, double * vx0, double * vy0, double * vz0, double viscosity)
  {
    // diffuse the velocity field (per axis):
    SWAP_PTR ( vx0, vx ); 
    diffuse ( N, vx, vx0, viscosity );
    SWAP_PTR ( vy0, vy ); 
    diffuse ( N, vy, vy0, viscosity );
    SWAP_PTR ( vz0, vz ); 
    diffuse ( N, vz, vz0, viscosity );
    // stabilize it: (vx0, vy0 are whatever, being used as temporaries to store gradient field)
    project ( N, vx, vy, vz, vx0, vy0 );
    
    // advect the velocity field (per axis):
    SWAP_PTR (vx0, vx); 
    SWAP_PTR (vy0, vy);
    SWAP_PTR (vz0, vz);
    advect ( N, vx, vx0, vx0, vy0, vz0 ); 
    advect ( N, vy, vy0, vx0, vy0, vz0 );
    advect ( N, vz, vz0, vx0, vy0, vz0 );
//    // stabilize it: (vx0, vy0 are whatever, being used as temporaries to store gradient field)
    project ( N, vx, vy, vz, vx0, vy0 );
  }

  #undef BEGIN_PER_CELL
  #undef END_PER_CELL
  #undef INDEX
  #undef SWAP_PTR

} // namespace

inline void Fluid :: step() {

  // diffuse pressure:
  stable::dens_step(dim, 
    pfield, pfield0, 
    uxfield, uyfield, uzfield, 
    diffusion );
    
  // diffuse velocity:
  stable::vel_step(dim, 
    uxfield0, uyfield0, uzfield0, 
    uxfield, uyfield, uzfield,    
    viscosity);
    
  // distort velocity:
  for (int i=0; i<dim3; i++) {
    uxfield[i] += srandom() * noise;
    uyfield[i] += srandom() * noise;
    uzfield[i] += srandom() * noise;
  }
    
  // decay velocity:
  for (int i=0; i<dim3; i++) {
    uxfield[i] *= decay;
    uyfield[i] *= decay;
    uzfield[i] *= decay;
  }
  
  // decay noise:
  for (int i=0; i<dim3; i++) {
    pfield[i] *= 0.9;
  }
  


 }

#endif