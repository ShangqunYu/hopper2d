#include "hopper2dEnv.hpp"

using namespace std;

Hopper2dEnv::Hopper2dEnv() : animator(parameter)
{

 }

void Hopper2dEnv::initstate(){
    state = p.init_state;

}

VectorXd Hopper2dEnv::reset(){
    state = p.init_state;
    return state;
}

VectorXd Hopper2dEnv::step(VectorXd actions){
    // compute torques based on actions with pd control
    vector<double> torques = compute_torques(actions);
    cout<<"torques: "<<torques[0]<<" "<<torques[1]<<" "<<torques[2]<<endl;
    // compute dz based on dynamics
    Eigen::VectorXd dz = dynamics(state, parameter, torques);
    cout<<"dz: "<<dz<<endl;
    // update state
    Eigen::VectorXd nexts = state + dz * p.dt;
    cout<<"nexts: "<<nexts<<endl;
    // compute collision, update velocity
    nexts.segment(p.dim/2, p.dim/2) = discrete_contact_dynamics(nexts, parameter, p.rest_coeff, p.fric_coeff, p.ground_height);
    cout<<"nexts: "<<nexts<<endl;
    // update position based on the corrected velocity after contact with old position
    nexts.segment(0, p.dim/2) = state.segment(0, p.dim/2) + nexts.segment(p.dim/2, p.dim/2) * p.dt;
    cout<<"nexts: "<<nexts<<endl;
    state = nexts;
    return state;
}


vector<double> Hopper2dEnv::compute_torques(VectorXd actions){
    // implement pd control
    VectorXd t = VectorXd::Zero(3);
    t = p.kp * (p.init_state.segment(3,3) + actions - state.segment(3, 3)) - p.kd * state.segment(9, 3);
    return {t[0], t[1], t[2]};
}

void Hopper2dEnv::render(){
    animator.render(state);
}
