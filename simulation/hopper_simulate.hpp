#pragma once
#include <casadi/casadi.hpp>
#include <stdlib.h> 
#include <cmath>
#include <Eigen/Core>
#include "dynamics.hpp"
#include "Animator.hpp"
#include "../utils/eigenHelper.hpp"
void hopper_simulate();

vector<double> basic_control(Eigen::VectorXd z0, const Eigen::Ref<const Eigen::MatrixXd>& z);