#include "../optimization/optimize2d.hpp"
#include "../utils/Parameters2d.hpp"
#include "../utils/State2d.hpp"
#include "../render/Animator2d.hpp"
#include <iostream>
#include <ctime>
using namespace std;

class hopper2dOpitEnv{
    public:
        hopper2dOpitEnv();
        ~hopper2dOpitEnv(){}
        double num_steps;
        Parameters2d p;
        State2d s;
        logdata log;
        Animator2d animator;
        State2d step(double contact_loc, double contact_dts, double fligth_dts);
        void reset();
        void render();
        void initstate();
        void generate_terrain();
        VectorXd terrain;
        bool checkinthepit(Vector2d rxs, Vector2d lxs);
        contact_data get_cs(int pred_hor, double dt, VectorXd dts, bool is_right);
        VectorXd get_terrian_obs();
        VectorXd terrain_obs;
        contact_data get_contact_data(contact_loc, contact_hor, flight_hor)
        MatrixXd get_desireX();
        double cur_contact_loc = 0;


};