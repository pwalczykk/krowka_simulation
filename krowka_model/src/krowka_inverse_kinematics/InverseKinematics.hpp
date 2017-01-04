#ifndef INVERSE_KINEMATTICS_HPP
#define INVERSE_KINEMATTICS_HPP

// l1 = 350 [mm]
// l2 = 275,5 [mm]
// l3 = 106 [mm]
// h = 70 [mm]
// s = 25 [mm]
//
// A - orientacja
// X, Y, Z - współrzędne efektora w przestrzeni
//
// x = (X + l3*sin(A)*cos(teta0))/cos((teta0)
// y = (Y + l3*sin(A)*sin(teta0))/sin(teta0)
// z = Z - l3*cos(A)

#include "MatlabVariable.hpp"
#include <stdio.h>

class InverseKinematics{
public:
    // CONSTATS
    MatlabVariable <double> l1, l2, l3, h, s, pi;

    // VARIABLES
    MatlabVariable <double> A, X, Y, Z;

    // DEPENDENT VARIABLES
    MatlabVariable <double> x, y ,z;

    // RESULTS
    MatlabVariable <double> theta0, theta1, theta2, theta3;

    MatlabVariable <double> temp1, temp2, temp3;


public:
    InverseKinematics(){
        l1.data = 350.0;
        l2.data = 275.5;
        l3.data = 106.0;
        h.data = 70.0;
        s.data = 25.0;
        pi.data = 3.1416;
    }

    ~InverseKinematics(){};

    void update(double set_A, double set_X, double set_Y, double set_Z){

        A.data = set_A;
        X.data = set_X;
        Y.data = set_Y;
        Z.data = set_Z;

        theta0 = atan2(Y, X);

        x = (X + l3*sin(A)*cos(theta0))/cos(theta0);
        y = (Y + l3*sin(A)*sin(theta0))/sin(theta0);
        z = Z - l3*cos(A);


        theta1 = 2*atan((2*l1*z - 2*h*l1 + (- h->*4 + 4*h->*3*z + 2*h->*2*l1->*2 + 2*h->*2*l2->*2 - 2*h->*2*s->*2 - 4*h->*2*s*x - 2*h->*2*x->*2 - 6*h->*2*z->*2 - 4*h*l1->*2*z - 4*h*l2->*2*z + 4*h*s->*2*z + 8*h*s*x*z + 4*h*x->*2*z + 4*h*z->*3 - l1->*4 + 2*l1->*2*l2->*2 + 2*l1->*2*s->*2 + 4*l1->*2*s*x + 2*l1->*2*x->*2 + 2*l1->*2*z->*2 - l2->*4 + 2*l2->*2*s->*2 + 4*l2->*2*s*x + 2*l2->*2*x->*2 + 2*l2->*2*z->*2 - s->*4 - 4*s->*3*x - 6*s->*2*x->*2 - 2*s->*2*z->*2 - 4*s*x->*3 - 4*s*x*z->*2 - x->*4 - 2*x->*2*z->*2 - z->*4)->*(1.0/2.0))/(h->*2 - 2*h*z + l1->*2 + 2*l1*s + 2*l1*x - l2->*2 + s->*2 + 2*s*x + x->*2 + z->*2));
        theta2 = -2*atan((2*h*l2 + (-(h->*2 - 2*h*z - l1->*2 - 2*l1*l2 - l2->*2 + s->*2 + 2*s*x + x->*2 + z->*2)*(h->*2 - 2*h*z - l1->*2 + 2*l1*l2 - l2->*2 + s->*2 + 2*s*x + x->*2 + z->*2))->*(1.0/2.0) - 2*l2*z)/(h->*2 - 2*h*z - l1->*2 + l2->*2 + 2*l2*s + 2*l2*x + s->*2 + 2*s*x + x->*2 + z->*2));
        theta3 = A - theta2 - 3*pi/2;

        // DEBUG
        // printf("%f\n",x.data);
        // printf("%f\n",y.data);
        // printf("%f\n",z.data);
        //
        // printf("%f\n",theta0.data);
        // printf("%f\n",theta1.data);
        // printf("%f\n",theta2.data);
        // printf("%f\n",theta3.data);
        //
        // printf("\n");
        //
        // temp1 = (2*l1*z - 2*h*l1 + (- h->*4 + 4*h->*3*z + 2*h->*2*l1->*2 + 2*h->*2*l2->*2 - 2*h->*2*s->*2 - 4*h->*2*s*x - 2*h->*2*x->*2 - 6*h->*2*z->*2 - 4*h*l1->*2*z - 4*h*l2->*2*z + 4*h*s->*2*z + 8*h*s*x*z + 4*h*x->*2*z + 4*h*z->*3 - l1->*4 + 2*l1->*2*l2->*2 + 2*l1->*2*s->*2 + 4*l1->*2*s*x + 2*l1->*2*x->*2 + 2*l1->*2*z->*2 - l2->*4 + 2*l2->*2*s->*2 + 4*l2->*2*s*x + 2*l2->*2*x->*2 + 2*l2->*2*z->*2 - s->*4 - 4*s->*3*x - 6*s->*2*x->*2 - 2*s->*2*z->*2 - 4*s*x->*3 - 4*s*x*z->*2 - x->*4 - 2*x->*2*z->*2 - z->*4)->*(1.0/2.0))/(h->*2 - 2*h*z + l1->*2 + 2*l1*s + 2*l1*x - l2->*2 + s->*2 + 2*s*x + x->*2 + z->*2);
        // temp2 = (- h->*4 + 4*h->*3*z + 2*h->*2*l1->*2 + 2*h->*2*l2->*2 - 2*h->*2*s->*2 - 4*h->*2*s*x - 2*h->*2*x->*2 - 6*h->*2*z->*2 - 4*h*l1->*2*z - 4*h*l2->*2*z + 4*h*s->*2*z + 8*h*s*x*z + 4*h*x->*2*z + 4*h*z->*3 - l1->*4 + 2*l1->*2*l2->*2 + 2*l1->*2*s->*2 + 4*l1->*2*s*x + 2*l1->*2*x->*2 + 2*l1->*2*z->*2 - l2->*4 + 2*l2->*2*s->*2 + 4*l2->*2*s*x + 2*l2->*2*x->*2 + 2*l2->*2*z->*2 - s->*4 - 4*s->*3*x - 6*s->*2*x->*2 - 2*s->*2*z->*2 - 4*s*x->*3 - 4*s*x*z->*2 - x->*4 - 2*x->*2*z->*2 - z->*4)->*(1.0/2.0);
        // printf("temp: %f\n",temp1.data);
        // printf("temp: %f\n",temp2.data);
        // printf("temp: %f\n",temp3.data);

    }
};

#endif
