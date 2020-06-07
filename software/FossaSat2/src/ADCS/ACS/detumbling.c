/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: detumbling.c
   04/30/20

   This file drives the detumbling procedure commands
*/

/************************* Headers **************************/
#include <math.h>
#include "adcs.h"

/********************** Main function ***********************/
void bdot_function(const double omega[], const double B[], const float meanOrbital, const float inclination, double I[])
{
    // Constants definition
    const float tol = 0.001;                                                                          // Tolerance to prevent NaN
    const double B_module = sqrt(B[0]*B[0]+B[1]*B[1]+B[2]*B[2])+tol;                                  // Module of the magnetic field intensity
    const double I_min = 100;                                                                         // Minimum inertial moment
    const float K = 2*meanOrbital*(1+sin(i))*I_min;                                                   // Controller gain constant
    const float a = K/(B_module*B_module);                                                            // General gain
    const double A_coils[3][3] = {{pow(1.88*pow(10,5),-1),0,0},
                                 {0,pow(6.1*pow(10,5),-1),0},
                                 {0,0,pow(5.96*pow(10,5),-1)}};   // Inverse matrix of coil magnetic characteristics

    // Generate the control law by means of a vector product: m = a*(B x omega) = (K/B_module)*(B x omega)
    float L[3];
    L[0] = a*(omega[1]*B[2]-omega[2]*B[1]);
    L[1] = a*(omega[2]*B[0]-omega[0]*B[2]);
    L[2] = a*(omega[0]*B[1]-omega[1]*B[0]);

    // Magnetic moment needed at the axes: calculated by minimization of a least squares problem
    float m[3];
    m[0] = (L[2]*B[1]-B[2]*L[1])/B_module;
    m[1] = (L[0]*B[2]-B[0]*L[2])/B_module;
    m[2] = (L[1]*B[0]-B[1]*L[0])/B_module;

    // Definition of intensity output -solving the equation: A*I = m-
    I[0] = m[0]*A[0][0]+m[1]*A[0][1]+m[2]*A[0][2];
    I[1] = m[0]*A[0][0]+m[1]*A[1][1]+m[2]*A[1][2];
    I[2] = m[0]*A[0][0]+m[1]*A[2][1]+m[2]*A[2][2];

    return;
}

