#include "hopper_simulate.h"

using namespace casadi;
using namespace std;
void hopper_simulate(){

    //robot property
    double mbody = 4; double m0 = 0.3; double m1 = 0.5; double m2 = 0.1;  // Mass
    double Ibody = 11 * pow(10,-3); double I0 = 5.1 * pow(10,-6); double I1 = 5.1 * pow(10,-6); double I2 = 1.5 * pow(10,-6); // inertia
    double l0 = 0.2; double l1 = 0.22; double l2 = 0.1;  double l21 = 0.06; double lbody = 0.3; // length of link
    double c0 = 0.1; double c1 = 0.1; double c2 = 0.07; // length to center of mass
    double gravity = 9.81; 

    int n = 40;
    double tf = 0.605;
    double sim_dt = 0.001;
    int num_sim_step = floor(tf/sim_dt);
    int dim = 12;
    vector<double> parameter = {mbody, Ibody, m0, m1, m2, I0, I1, I2, c0, c1, c2, l0, l1, l2, l21, gravity, lbody};
    Eigen::VectorXd z0(12); z0<< 0.0, 0.4, 0.0, M_PI/6, -M_PI/3, M_PI/6, 0.0, 0.0,  0.0,  0.0,  0.0, 0.0; 
    std::cout << "z0\n" << z0<< '\n';  

    Eigen::MatrixXd z_out(dim,num_sim_step);
    vector<double> taus = {0,0,0};
    z_out.block(0,0,12,1) = z0;
    std::cout << "z_out " << z_out.block(0,0,12,12)<< '\n';  
    for(int i = 0; i < 1; i++){
        dynamics(z_out.block(0,i,12,1), parameter, taus);
        
    }
    
    
}