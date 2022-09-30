#include "hopper_simulate.hpp"

using namespace casadi;
using namespace std;
void hopper_simulate(){


    //robot property
    double mbody = 4; double m0 = 0.3; double m1 = 0.5; double m2 = 0.1;  // Mass
    double Ibody = 11 * pow(10,-3); double I0 = 5.1 * pow(10,-6); double I1 = 5.1 * pow(10,-6); double I2 = 1.5 * pow(10,-6); // inertia
    double l0 = 0.2; double l1 = 0.22; double l2 = 0.1;  double l21 = 0.06; double lbody = 0.3; // length of link
    double c0 = 0.1; double c1 = 0.1; double c2 = 0.07; // length to center of mass
    double gravity = 9.81; 
    double ground_height = 0;
    double rest_coeff = 0.2; 
    double fric_coeff = 0.7;

    int n = 40;
    double tf = 0.605;
    double sim_dt = 0.001;
    int num_sim_step = floor(tf/sim_dt);
    int dim = 12;
    vector<double> parameter = {mbody, Ibody, m0, m1, m2, I0, I1, I2, c0, c1, c2, l0, l1, l2, l21, gravity, lbody};
    Eigen::VectorXd z0(dim); z0<< 0.0, 0.4, 0.0, M_PI/6, -M_PI/3, M_PI/6, 0.0, 0.0,  0.0,  0.0,  0.0, 0.0; 
    z0(1) = l0*cos(z0(3)) + l1*cos(z0(3)+z0(4)) + ground_height + lbody/2;
    Eigen::MatrixXd z_out(dim,num_sim_step);
    vector<double> taus = {0,0,0};
    z_out.block(0,0,dim,1) = z0;
    for(int i = 0; i < num_sim_step-1; i++){
        taus =  basic_control(z0, z_out.block(0,i,dim,1));
        Eigen::VectorXd dz = dynamics(z_out.block(0,i,dim,1), parameter, taus);
        
        z_out.block(0,i+1,dim,1) = z_out.block(0,i,dim,1) + dz * sim_dt;
        z_out.block(dim/2,i+1,dim/2,1)= discrete_contact_dynamics(z_out.block(0,i+1,dim,1), parameter, rest_coeff, fric_coeff, ground_height);
        z_out.block(0,i+1,dim/2,1) = z_out.block(0,i,dim/2,1) + z_out.block(dim/2,i+1,dim/2,1) * sim_dt;
        //cout<< "i: "<<i<<endl;
        //break;
    }
    
    //cout<< z_out.block(0,num_sim_step-20,dim,10);
    Animator animator(parameter);

    animator.animate(z_out);
    

}

vector<double> basic_control(Eigen::VectorXd z0, const Eigen::Ref<const Eigen::MatrixXd>& z){
    double k_th = 50;
    double D_th = 0.05;
    Eigen::VectorXd q_des = z0.block(3,0,3,1);
    Eigen::VectorXd q_cur =  z.block(3,0,3,1);
    Eigen::VectorXd qdot  = z.block(0,0,3,1);
    Eigen::VectorXd tau_ = k_th * (q_des - q_cur) + D_th *(- qdot);
    vector<double> tau(tau_.data(), tau_.data() + tau_.rows() * tau_.cols());
    return tau;
}