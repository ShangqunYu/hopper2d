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

Function central_model_dynamics(double mobody, double m0, double m1, double m2, double Ibody, double gravity, double ground_height);