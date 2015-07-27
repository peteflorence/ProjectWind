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

using namespace Eigen;
using namespace std;


template <typename DerivedX, typename DerivedU>
VectorXd quadDynamics(const mxArray* pobj, const double &t, const MatrixBase<DerivedX> &x, const MatrixBase<DerivedU> &u)
{
  VectorXd xdot(13);
  double m  = mxGetScalar(mxGetProperty(pobj,0,"m"));
  auto I = matlabToEigenMap<3,3>(mxGetProperty(pobj,0,"I"));
  auto invI = matlabToEigenMap<3,3>(mxGetProperty(pobj,0,"invI"));
  double g = 9.81;
  double L = 0.1750;

  double phi = x(3);      // 0-indexing in C
  double theta = x(4);
  double psi = x(5);
  double phidot = x(10);
  double thetadot = x(11);
  double psidot = x(12);

  double w1 = u(1);
  double w2 = u(2);
  double w3 = u(3);
  double w4 = u(4);



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