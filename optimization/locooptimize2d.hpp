#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include "../utils/eigenHelper.hpp"
#include "../utils/LocoParams.hpp"
#include "../utils/State2d.hpp"
using namespace std;
using namespace casadi;

struct contact_data
{
    Eigen::MatrixXd  rcl;  // location of joint between right foot and shank
    Eigen::MatrixXd  lcl;  // location of joint between left foot and shank
    vector<int>  rcs;  // contact status of right foot
    vector<int>  lcs;  // contact status of left foot
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
    DM rtoef;
    DM rheelf;
    DM ltoef;
    DM lheelf;
};
logdata locooptimize2d(LocoParams p, State2d s, contact_data cdata, MatrixXd xk_des);
