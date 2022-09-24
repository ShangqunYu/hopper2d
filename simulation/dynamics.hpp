#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <typeinfo>
#include "../utils/eigenHelper.hpp"
#include "../matlab_gen/b_hopper.h"

using namespace std;

//template <typename Derived>
Eigen::VectorXd dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus);