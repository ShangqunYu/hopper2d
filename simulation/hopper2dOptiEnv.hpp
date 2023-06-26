#include "../optimization/optimize2d.hpp"
#include "../utils/Parameters.hpp"
#include "../utils/State2d.hpp"
#include "../render/Animator.hpp"
#include <iostream>
#include <ctime>
using namespace std;

class hopper2dOptiEnv{
    public:
        hopper2dOptiEnv();
        ~hopper2dOptiEnv(){}
        double num_steps;
        Parameters2d p;
        State2d s;
        logdata log;
        // Animator2d animator;
        State2d step(double contact_loc, double contact_dts, double fligth_dts);
        void reset();
        // void render();
        void initstate();
        contact_data get_contact_data(contact_loc, contact_hor, flight_hor)
        MatrixXd get_desireX();


};