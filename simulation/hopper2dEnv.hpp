#include "../utils/Parameters.hpp"
#include "dynamics.hpp"
#include "../render/Animator.hpp"
#include "../matlab_gen/position_contact_points.h"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class Hopper2dEnv{
    public:
        Hopper2dEnv();
        ~Hopper2dEnv(){}
        int num_steps;
        Parameters p;
        vector<double> parameter = {p.mbody, p.Ibody, p.m0, p.m1, p.m2, p.I0, p.I1, p.I2, p.c0, p.c1, p.c2, p.l0, p.l1, p.l2, p.l21, p.gravity, p.lbody};
        VectorXd state;
        Animator animator;
        VectorXd step(VectorXd actions);
        VectorXd forward(VectorXd actions);
        vector<double> torques;
        double calc_stand_reward();
        double calc_jump_reward();
        VectorXd reset();
        VectorXd action_avg;
        void render();
        void initstate();
        vector<double> compute_torques(VectorXd actions);
        bool is_done();
        double prev_x;
        double prev_height;

};