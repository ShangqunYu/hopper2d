#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include "../utils/eigenHelper.hpp"
#include "../utils/State.hpp"
#include "../utils/Control.hpp"
#include "../utils/Parameters.hpp"
#include "../utils/State2d.hpp"
#include "build_desire_state.hpp"
using namespace std;
using namespace casadi;

struct contact_data
{
    Eigen::MatrixXd  rcl;  // right contact location
    Eigen::MatrixXd  lcl;  // left contact location
    vector<int>  cs;  // contact status
};
struct logdata
{
    bool done;
    double reward;
    contact_data rcs;
    contact_data lcs;
    DM x;
    DM xd;
    DM theta;
    DM w;
    DM rf;
    DM lf;
    Eigen::MatrixXd rc;
    Eigen::MatrixXd lc;
};
logdata optimize2d(Parameters2d p, State2d s, contact_data cdata, MatrixXd xk_des);
