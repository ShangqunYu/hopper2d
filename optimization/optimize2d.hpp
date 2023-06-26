#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include "../utils/eigenHelper.hpp"
#include "../utils/Parameters.hpp"
#include "../utils/State2d.hpp"
using namespace std;
using namespace casadi;

struct contact_data
{
    Eigen::MatrixXd  cl;  // location of joint between foot and shank
    vector<int>  cs;  // contact status
};
struct logdata
{
    bool done;
    double reward;
    contact_data cd;
    DM x;
    DM xd;
    DM theta;
    DM w;
    DM cf;
    DM ef;
};
logdata optimize2d(Parameters p, State2d s, contact_data cdata, MatrixXd xk_des);
