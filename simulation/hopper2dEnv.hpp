#include "../utils/Parameters.hpp"
#include <Eigen/Core>

using namespace std;

class Hopper2dEnv{
    public:
        Hopper2dEnv();
        ~Hopper2dEnv(){}
        double num_steps;
        Parameters p;
        State2d s;
        logdata log;
        Animator2d animator;
        State2d step(VectorXd rdts, VectorXd ldts, Vector2d rxs, Vector2d lxs);
        void reset();
        void render();
        void initstate();
        void generate_terrain();
        VectorXd terrain;
        bool checkinthepit(Vector2d rxs, Vector2d lxs);
        contact_data get_cs(int pred_hor, double dt, VectorXd dts, bool is_right);
        VectorXd get_terrian_obs();
        VectorXd terrain_obs;
        Vector2d get_contacty(Vector2d xs);
        MatrixXd get_desireX();


};