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
    vector<double> torques = compute_torques(actions);
    Eigen::VectorXd dz = dynamics(state, parameter, torques);
    Eigen::VectorXd nexts = state + dz * p.dt;
    nexts.segment(p.dim/2, p.dim/2) = nexts.segment(0, p.dim/2) + discrete_contact_dynamics(nexts, parameter, p.rest_coeff, p.fric_coeff, p.ground_height);
    nexts.segment(0, p.dim/2) = state.segment(0, p.dim/2) + nexts.segment(p.dim/2, p.dim/2) * p.dt;
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
