#include "hopper2dEnv.hpp"

using namespace std;

Hopper2dEnv::Hopper2dEnv():
    animator(parameter)
{
    parameter = {p.mbody, p.Ibody, p.m0, p.m1, p.m2, p.I0, p.I1, p.I2, p.c0, p.c1, p.c2, p.l0, p.l1, p.l2, p.l21, p.gravity, p.lbody};
    

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
