#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <typeinfo>
#include "../utils/dmToEigen.h"
using namespace std;

//template <typename Derived>
void dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus);