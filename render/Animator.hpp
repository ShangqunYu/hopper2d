#ifndef ANIMATE_H
#define ANIMATE_H

#include <unistd.h>
#include <Eigen/Core>
#include <iostream>
#include <casadi/casadi.hpp>
#include "../utils/eigenHelper.hpp"
#include <cart_pole_lcmt.hpp>
#include <hopper2d_lcmt.hpp>
#include <lcm/lcm-cpp.hpp>
using namespace casadi;
using namespace std;

class Animator{
    public:
        Animator(vector<double> p);
        ~Animator(){}
        void animate(Eigen::MatrixXd z);
        void send_message(); 
        lcm::LCM _lcm_cart;
        hopper2d_lcmt _kin_msg;   
    private:
        int _dim;
        double lbody;
        double l0;
        double l1;
        double l2;
        double l21;
        Eigen::VectorXd _ini_state;
        Eigen::VectorXd _cur_state;
};




#endif