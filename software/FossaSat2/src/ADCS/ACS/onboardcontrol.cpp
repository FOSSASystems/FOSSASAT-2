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
    const double B_module = pow((pow(magnetic_vector[0],2)+pow(magnetic_vector[1],2)+magnetic_vector[2]),(1/2));
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

    //Calculation of magnetic dipole applied on the coils
    float m[3];
    m[0] = (Lc[1]*B[2]-Lc[2]*B[1])/ B_module;
    m[1] = (Lc[2]*B[0]-Lc[0]*B[2])/ B_module;
    m[2] = (Lc[0]*B[1]-Lc[1]*B[0])/ B_module;

    //Definition of intensity output -solving the equation: A*I = m-
    I[0] = m[0]/A[0][0];
    I[1] = m[1]/A[1][1];
    I[2] = m[2]/A[2][2];

    return;

}*/
