#pragma once
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <vector>
using namespace std;
using namespace casadi;
Eigen::MatrixXd dmToEigen(casadi::DM &m);

Eigen::MatrixXd arrayToEigen(double m[], int row , int col);
casadi::DM EigenTodm(Eigen::MatrixXd matrix);
casadi::DM EigenMatrixTodm(Eigen::MatrixXd matrix);
casadi::DM EigenVectorTodm(Eigen::VectorXd vec);