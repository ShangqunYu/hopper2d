#include "../optimization/optimize2d.hpp"
#include "../utils/Parameters.hpp"
#include "../utils/State2d.hpp"
#include "../render/AnimatorOpti.hpp"
#include <iostream>
#include <ctime>
using namespace std;

class hopper2dOptiEnv{
    public:
        hopper2dOptiEnv();
        ~hopper2dOptiEnv(){}
        double num_steps;
        Parameters p;
        State2d s;
        logdata log;
        AnimatorOpti animator;
        State2d step(double contact_loc, double contact_dts, double fligth_dts);
        State2d reset();
        void render();
        void initstate();
        contact_data get_contact_data(double contact_loc, int contact_hor, int flight_hor);
        MatrixXd get_desireX();
        int pred_hor;


};