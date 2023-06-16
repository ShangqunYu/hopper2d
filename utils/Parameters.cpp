#include "Parameters.hpp"

Parameters::Parameters() 
{
    mbody = 6; m0 = 0.3; m1 = 0.5; m2 = 0.1;  
    M = mbody + m0 + m1 + m2; // Total Mass

    Ibody = 11 * pow(10,-3); I0 = 5.1 * pow(10,-6); I1 = 5.1 * pow(10,-6); I2 = 1.5 * pow(10,-6); // inertia

    l0 = 0.2; l1 = 0.22; l2 = 0.1; l21 = 0.06; lbody = 0.3; // length of link

    c0 = 0.1; c1 = 0.1;  c2 = 0.07; // length to center of mass

    gravity = 9.81; 

    ground_height = 0;
        
    rest_coeff = 0; 
    fric_coeff = 0.8;
    dt = 0.001;
    dim = 12;
    // declare a 12 dim eigen vector
    init_state = Eigen::VectorXd::Zero(dim);
    init_state << 0.0, 1, 0.0, M_PI/6, -M_PI/3, M_PI/6, 0.0, 0.0,  0.0,  0.0,  0.0, 0.0; 
    init_state(1) = l0*cos(init_state(3)) + l1*cos(init_state(3)+init_state(4)) + ground_height + lbody/2;

    kp = 100;
    kd = 10;

    max_torque << 40, 40, 40;

    terminal_height = -11110;

    max_steps = 10000;
};