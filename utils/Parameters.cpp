#include "Parameters.hpp"

Parameters::Parameters() 
{
    mbody = 8; m0 = 2.7; m1 = 0.9; m2 = 0.3;  
    M = mbody + m0 + m1 + m2; // Total Mass



    l0 = 0.35; l1 = 0.36; l2 = 0.1; l21 = 0.06; lbody = 0.3; // length of link


    Ibody = mbody * lbody * lbody / 6; I0 = m0 * l0 * l0 / 12; I1 = m1 * l1 * l1 / 12; I2 = m2 * (l2 + l21) * (l2 + l21) / 12; // inertia
    c0 = 0.13; c1 = 0.18;  c2 = 0.02; // length to center of mass

    gravity = 9.81; 

    ground_height = 0;
        
    rest_coeff = 0; 
    fric_coeff = 0.8;
    dt = 0.002;
    dim = 12;
    // declare a 12 dim eigen vector
    init_state = Eigen::VectorXd::Zero(dim);
    init_state << 0.0, 1, 0.0, M_PI/6, -M_PI/3, M_PI/6, 0.0, 0.0,  0.0,  0.0,  0.0, 0.0; 
    init_state(1) = l0*cos(init_state(3)) + l1*cos(init_state(3)+init_state(4)) + ground_height + lbody/2;

    kp = 15;
    kd = 1.5 ;

    max_torque << 70, 70, 70;

    terminal_height = 0.28;
    terminal_width = 0.1;
    terminal_angle = M_PI / 6;
    terminal_theta1 = M_PI / 2;
    terminal_theta2 = 2 * M_PI / 3;
    terminal_theta3 = M_PI / 2;
    healthy_state_range = 100;

    max_steps = 5000;

    time_skipping = 5;
};