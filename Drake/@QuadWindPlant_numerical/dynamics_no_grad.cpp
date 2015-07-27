/*********************************************************************
 * dynamics_no_grad.cpp
 *
 * This file shows the basics of setting up a mex file to work with
 * Matlab.  This example shows how to use 2D matricies.  This may
 *
 * Keep in mind:
 * <> Use 0-based indexing as always in C or C++
 * <> Indexing is column-based as in Matlab (not row-based as in C)
 * <> Use linear indexing.  [x*dimy+y] instead of [x][y]
 *
 *
 ********************************************************************/
#include "mex.h"
#include <Eigen/Dense>
#include <cmath>
#include "drakeMexUtil.h"
#include <iostream>
#include "matrix.h"
#include "drakeGradientUtil.h"
#include "drakeGeometryUtil.h"

using namespace Eigen;
using namespace std;





// MATLAB version is
//windout = obj.quadwind(quadpos,x(13),1); % pass mytime to quadwind. % last arument is plot option

Vector3d quadWind(const mxArray* pobj, Vector3d quadpos, double time)
{
  Vector3d wind;

  double xquad = quadpos(0);
  double yquad = quadpos(1);
  double zquad = quadpos(2);

  double V_0 = 3.5;
  double c = 0.1;
  double V = V_0 / (1 + V_0 * c * time);


  auto ellipsoidcenter = matlabToEigenMap<3,1>(mxGetProperty(pobj,0,"ellipsoidcenter")).eval();

  Vector3d timevec;
  timevec << V*time, 0, 0;

  ellipsoidcenter = ellipsoidcenter - timevec;

  double xcenter = ellipsoidcenter(0);
  double ycenter = ellipsoidcenter(1);
  double zcenter = ellipsoidcenter(2);

  double sphereRadius = 0.30;
  double nomwind = -5.0;

  double xwind = 0.0;
  double ywind = 0.0;
  double zwind = 0.0;

  double xidif = xquad - xcenter;
  double yidif = yquad - ycenter;
  double zidif = zquad - zcenter;

  double scale = nomwind;
  double reversed = -1.0;
  double a = sqrt(pow(xidif,2) + pow(yidif,2) + pow(zidif,2));
  double slope = 10.0;

  xwind = scale * (tanh(reversed * ( a - sphereRadius) * slope ) +1) / 2;

  wind << xwind, ywind, zwind;

  return wind;

}

template <typename DerivedX, typename DerivedU>
VectorXd quadDynamics(const mxArray* pobj, const double &t, const MatrixBase<DerivedX> &x, const MatrixBase<DerivedU> &u)
{
  VectorXd xdot(13);
  double m  = mxGetScalar(mxGetProperty(pobj,0,"m"));
  auto I = matlabToEigenMap<3,3>(mxGetProperty(pobj,0,"I"));

  auto invI = I.inverse().eval();

  double g = 9.81;
  double L = 0.1750;

  double phi = x(3);      // 0-indexing in C
  double theta = x(4);
  double psi = x(5);
  double phidot = x(9);
  double thetadot = x(10);
  double psidot = x(11);

  double w1 = u(0);
  double w2 = u(1);
  double w3 = u(2);
  double w4 = u(3);


// My own rpy2rotmat implementation
//  double cos_r = cos(phi);
//  double sin_r = sin(phi);
//  double cos_p = cos(theta);
//  double sin_p = sin(theta);
//  double cos_y = cos(psi);
//  double sin_y = sin(psi);
//  Matrix3d rotMat;
//  rotMat << cos_y*cos_p, cos_y*sin_p*sin_r-sin_y*cos_r, cos_y*sin_p*cos_r+sin_y*sin_r,
//  sin_y*cos_p, sin_y*sin_p*sin_r+cos_y*cos_r, sin_y*sin_p*cos_r-cos_y*sin_r,
//  -sin_p, cos_p*sin_r, cos_p*cos_r;
//

  Vector3d rpy;
  rpy << phi, theta, psi;
  auto R = rpy2rotmat(rpy);


  double result = tanh (1.0);



  double kf = 1; // 6.11*10^-8;

  double F1 = kf*w1;
  double F2 = kf*w2;
  double F3 = kf*w3;
  double F4 = kf*w4;

  double km = 0.0245;

  double M1 = km*w1;
  double M2 = km*w2;
  double M3 = km*w3;
  double M4 = km*w4;

  Vector3d quadpos;
  quadpos << x(0), x(1), x(2);

  Vector3d windout = quadWind(pobj, quadpos, t);





  xdot << m, invI(1,1), invI(2,2), phi*10, g, 6, 7, 8, 9, 10, 11, 12, 13;

  return xdot;


}




// Structure from MATLAB
// dynamics_no_grad(obj,t,x,u)

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray* pobj = prhs[0];

  if (mxIsDouble(prhs[1]) && mxIsDouble(prhs[2]) && mxIsDouble(prhs[3]) ) {

    double t =  mxGetScalar(prhs[1]);
    auto x = matlabToEigenMap<13,1>(prhs[2]);
    auto u = matlabToEigenMap<4,1>(prhs[3]);

    VectorXd xdot = quadDynamics(pobj, t, x, u);

    plhs[0] = eigenToMatlab(xdot);

  }

    return;
}