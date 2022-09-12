#include <cmath>
#include <iostream>
#include <string>
#include <stdlib.h> 
#include "Hopper2dConfig.h"
#include "animate.h"


#include <filesystem>
#include <hopper_simulate.h>
//#include "gen.h"

namespace fs = std::filesystem;
using namespace std;
int main(int argc, char* argv[]){
    

    std::cout << "hello world current version is: "<< hopper2d_VERSION_MAJOR << "."<< hopper2d_VERSION_MINOR << std::endl;

    std::cout << "Current path is " << fs::current_path() << '\n';

    hopper_simulate();

    animate();
    return 0;
}