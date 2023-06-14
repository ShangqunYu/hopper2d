#include "../utils/Parameters.hpp"
#include "dynamics.hpp"
#include "Animator.hpp"

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class Hopper2dEnv{
    public:
        Hopper2dEnv();
        ~Hopper2dEnv(){}
        int num_steps;
        Parameters p;
        vector<double> parameter; 
        VectorXd state;
        Animator animator;
        VectorXd step(VectorXd actions);
        VectorXd reset();
        void render();
        void initstate();
        vector<double> compute_torques(VectorXd actions);


};