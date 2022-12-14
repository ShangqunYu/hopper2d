#pragma once
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <vector>
using namespace std;
using namespace casadi;
Eigen::MatrixXd dmToEigen(vector<DM> &m);

Eigen::MatrixXd arrayToEigen(double m[], int row , int col);
casadi::DM EigenTodm(Eigen::MatrixXd matrix);
void print_hello();