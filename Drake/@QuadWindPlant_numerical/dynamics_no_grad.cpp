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

using namespace Eigen;
using namespace std;


template <typename DerivedA, typename DerivedC>
void manipulatorDynamics(const mxArray* pobj, const MatrixBase<DerivedA> &t, MatrixBase<DerivedC> &c_t)
{
  c_t << 27.0;

}


// Structure from MATLAB
// dynamics_no_grad(obj,t,x,u)

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  const mxArray* pobj = prhs[0];

  if (mxIsDouble(prhs[1]) && mxIsDouble(prhs[2]) && mxIsDouble(prhs[3]) ) {
    //mexPrintf("Does that check right? = ");
    auto t = matlabToEigenMap<1,1>(prhs[1]);
    mexPrintf("Can we get here?");

    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
    Map<Vector2d> c_t(mxGetPr(plhs[0]));

    manipulatorDynamics(pobj, t, c_t);

//    auto qd = matlabToEigenMap<2,1>(prhs[2]);
//
//    plhs[0] = mxCreateDoubleMatrix(2,2,mxREAL);
//    Map<Matrix2d> H(mxGetPr(plhs[0]));
//    plhs[1] = mxCreateDoubleMatrix(2,1,mxREAL);
//    Map<Vector2d> C(mxGetPr(plhs[1]),2);
//    plhs[2] = mxCreateDoubleMatrix(2,1,mxREAL);
//    Map<Vector2d> B(mxGetPr(plhs[2]),2);
//
//    manipulatorDynamics(pobj,q,qd,H,C,B);
  }

    return;
}