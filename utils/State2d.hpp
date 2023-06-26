#pragma once
#include <Eigen/Core>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;
class State2d{
    public:
        State2d(Vector2d x, Vector2d xd, double theta, double w, double reward, double curr_contact_loc);
        State2d();
        ~State2d(){}
        //hyper parameters for mpc
        Vector2d x;
        Vector2d xd;
        double theta;   //orientation
        double w;   //angular velocity
        double reward;
        void operator = (const State2d &s); 
        double curr_contact_loc;   //current contact location relative to the com on x axis 

};