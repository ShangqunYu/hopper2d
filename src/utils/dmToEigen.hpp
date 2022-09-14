#pragma once
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <vector>
using namespace std;
using namespace casadi;
Eigen::MatrixXd dmToEigen(vector<DM> &m);


void print_hello();