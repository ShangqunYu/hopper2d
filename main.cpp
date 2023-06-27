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
    env.step(0.4, 0.5, 0.5);
    return 0;
}