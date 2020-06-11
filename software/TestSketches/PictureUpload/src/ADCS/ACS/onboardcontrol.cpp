/* Project: FossaSat-2 Flight Software
   Author: Team Steiner
   Module: adcs.h
   File: onboardcontrol.c
   04/30/20

   This file drives the main active control algorithm and its calculations
*/

/******************** Headers **********************/
#include <math.h>
#include "../ADCS/adcs.h"

/****************** Main function *********************/
/*void onboardcontrol(double state_variables[], double magnetic_vector[], double K[][], double I[])
{
    // Constants definition
    const int number_axes = 3;          // Number of controlled axes, in this case the 3 of them
    const int number_variables = 6;     // Number of state variables: 3 angles and 3 angular velocities

    //Module of the magnetic field intensity
    const float tol = 0.01;
    const double B_module = sqrt(magnetic_vector[0]*magnetic_vector[0]+magnetic_vector[1]*magnetic_vector[1]
                                +magnetic_vector[2]*magnetic_vector[2])+tol;
    //Coil magnetic characteristics
    const double A[3][3] = {{1.88*pow(10,5),0,0}, {0,6.1*pow(10,5),0}, {0,0,5.96*pow(10,5)}};

    // Variables initialization
    float Lc[3], Lc_aux = 0;

   // Generation of the control law by means of a matrix product Lc = K*M
    for (int i = 0; i <= number_axes; i++)
    {
        for (int j = 0; j <= number_variables; j++)
        {
          Lc_aux += K[i][j]*state_variables[j];
          Lc[i] = Lc_aux;
        }
    }

    // Calculation of magnetic dipole applied on the coils
    float m[3];
    m[0] = (L[2]*B[1]-B[2]*L[1])/pow(B_module,2);
    m[1] = (L[0]*B[2]-B[0]*L[2])/pow(B_module,2);
    m[2] = (L[1]*B[0]-B[1]*L[0])/pow(B_module,2);

    // Definition of intensity output -solving the equation: A*I = m-
    I[0] = m[0]*A[0][0]+m[1]*A[0][1]+m[2]*A[0][2];
    I[1] = m[0]*A[1][0]+m[1]*A[1][1]+m[2]*A[1][2];
    I[2] = m[0]*A[2][0]+m[1]*A[2][1]+m[2]*A[2][2];

    return;

}*/
