#pragma once
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <vector>
using namespace std;
using namespace casadi;
Eigen::MatrixXd dmToEigen(vector<DM> &m);

Eigen::MatrixXd arrayToEigen(double m[], int row , int col);
void print_hello();