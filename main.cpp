#include <cmath>
#include <iostream>
#include <string>
#include <stdlib.h> 
#include <Eigen/Core>
#include "Hopper2dConfig.h"
#include "Animator.hpp"
#include "CartPoleDynamics.hpp"
#include "simulation/hopper2dOptiEnv.hpp"
#include <filesystem>
#include <hopper_simulate.hpp>
#include <hopper2dEnv.hpp>
//#include "gen.h"

namespace fs = std::filesystem;
using namespace std;
int main(int argc, char* argv[]){

    hopper2dOptiEnv env;
    env.reset();
    State2d s = env.step(0.4, 0.5, 0.5);

    State2d s1 = env.reset();

    cout << "s1.x: " << s1.x << endl;
    cout << "s1.xd: " << s1.xd << endl;
    cout << "s1.theta: " << s1.theta << endl;
    cout << "s1.w: " << s1.w << endl;
    cout << "s1.reward: " << s1.reward << endl;
    cout << "s1.curr_contact_loc: " << s1.curr_contact_loc << endl;

    return 0;
}