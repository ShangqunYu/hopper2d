#ifndef ANIMATEOPTI_H
#define ANIMATEOPTI_H

#include <unistd.h>
//#include <SFML/Graphics.hpp>
#include <Eigen/Core>
#include <iostream>
#include <casadi/casadi.hpp>
#include "../utils/eigenHelper.hpp"
#include "../optimization/optimize2d.hpp"
#include <hopper2dOpti_lcmt.hpp>
#include <lcm/lcm-cpp.hpp>
using namespace casadi;
using namespace std;

class AnimatorOpti{
    public:
        AnimatorOpti();
        ~AnimatorOpti(){}
        void animate(logdata log);
        void send_message(double x, double y, double theta, double contact_loc, double in_contact); 
        lcm::LCM _lcm_cart;
};




#endif
