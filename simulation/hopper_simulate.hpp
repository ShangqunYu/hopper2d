#pragma once
#include <casadi/casadi.hpp>
#include <stdlib.h> 
#include <cmath>
#include <Eigen/Core>
#include "dynamics.hpp"
#include "../render/Animator.hpp"
#include "../utils/eigenHelper.hpp"
#include "../matlab_gen/jacobian_b.h"

void hopper_simulate();

vector<double> basic_control(Eigen::VectorXd z0, const Eigen::Ref<const Eigen::MatrixXd>& z);

casadi::SX centroidal_dynamics(SX state, SX Fx, SX Fy, SX Tau, double M,  double Inertia, double gravity, double ground_height);
