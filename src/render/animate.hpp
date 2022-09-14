#ifndef ANIMATE_H
#define ANIMATE_H


#include <SFML/Graphics.hpp>
#include <Eigen/Core>
#include <iostream>
#include <casadi/casadi.hpp>
#include "../utils/dmToEigen.hpp"
using namespace casadi;
using namespace std;

void animate(Eigen::MatrixXd z, vector<double> p);
//void test();

#endif