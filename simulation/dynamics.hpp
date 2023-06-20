#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <typeinfo>
#include "../utils/eigenHelper.hpp"
#include "../matlab_gen/A_hopper.h"
#include "../matlab_gen/b_hopper.h"
#include "../matlab_gen/position_contact_points.h"
#include "../matlab_gen/velocity_contact_points.h"
#include "../matlab_gen/jacobians_manager.hpp"
using namespace std;

//template <typename Derived>
Eigen::VectorXd dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus);


Eigen::VectorXd discrete_contact_dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, double rest_coeff, double fric_coeff, double ground_height);
Eigen::VectorXd discrete_contact_dynamics_new(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, double rest_coeff, double fric_coeff, double ground_height);

Eigen::MatrixXd get_A_matrix (const double z_vecp[12], const double paramp[17]);
Eigen::MatrixXd get_b_matrix (const double z_vecp[12], const double tausp[3], const double paramp[17]);

Eigen::VectorXd rk4(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus, double dt);