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


template <typename DerivedA>
VectorXd quadDynamics(const mxArray* pobj, const double &t, const MatrixBase<DerivedA> &x, const MatrixBase<DerivedA> &u)
{
  VectorXd xdot(13);
  xdot << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13;
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