#include "../optimization/locooptimize2d.hpp"
#include "../utils/LocoParams.hpp"
#include "../utils/State2d.hpp"
#include "../render/AnimatorOpti.hpp"
#include <iostream>
#include <ctime>
using namespace std;

class loco2dOptiEnv{
    public:
        loco2dOptiEnv();
        ~loco2dOptiEnv(){}
        double num_steps;
        LocoParams p;
        State2d s;
        loco_logdata log;
        AnimatorOpti animator;
        State2d step(double r_contact_loc, double r_contact_dts, double r_flight_dts, double l_contact_loc, double l_flight_dts, double l_contact_dts);
        State2d reset();
        void render();
        void initstate();
        loco_con_data get_contact_data(double r_contact_loc, int r_contact_hor, int r_flight_hor, double l_contact_loc, int l_flight_hor, int l_contact_hor);
        MatrixXd get_desireX();
        int pred_hor;
        double prev_x;
        double calc_reward();


};