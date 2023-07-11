#pragma once
#include <Eigen/Core>
#include <casadi/casadi.hpp>

using namespace casadi;
class LocoParams {
    public:
        LocoParams();
        ~LocoParams(){}

        //robot property
        double mbody, m0, m1, m2, M;  

        double Ibody, I0, I1, I2; // inertia

        double l0, l1, l2, l21, lbody; // length of link
        Eigen::Vector2d c_to_b;
        Eigen::Vector2d e_to_b;

        double c0, c1, c2; // length to center of mass

        double gravity; 

        double ground_height;
        
        double rest_coeff; 
        double fric_coeff;
        double dt;
        int dim;
        // declare a 12 dim eigen vector
        Eigen::VectorXd init_state;

        double kp, kd;

        Eigen::Vector3d max_torque;

        double terminal_height;
        double terminal_width;
        double terminal_angle;
        double terminal_theta1;
        double terminal_theta2;
        double terminal_theta3;
        double healthy_state_range;

        int max_steps;
        int time_skipping;

        //cost weight
        DM QX;   //position
        DM QX_terminal;  // terminal position
        DM QXd;  //velocity
        DM QXd_terminal;  // terminal velocity
        double QTheta;   //orientation
        double QW;   //angular velocity
        DM QC;   //control cost

        double opt_dt;
        //desire state
        DM xk_des;
        DM xdk_des;
        double thetak_des;
        double wk_des;    

        DM upper_bdbox;
        DM lower_bdbox;
        DM fpose;
        double theta_max;
        double max_react_force;
        DM gravity_opti;
        int max_iter;

        double min_dist;
};