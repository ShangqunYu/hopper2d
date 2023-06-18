#include "hopper2dEnv.hpp"

using namespace std;

Hopper2dEnv::Hopper2dEnv() : animator(parameter)
{

 }

void Hopper2dEnv::initstate(){
    state = p.init_state;

}

VectorXd Hopper2dEnv::reset(){
    num_steps = 0;
    state = p.init_state;
    return state;
}

VectorXd Hopper2dEnv::step(VectorXd actions){
    // compute torques based on actions with pd control
    torques = compute_torques(actions);
    // compute dz based on dynamics
    Eigen::VectorXd dz = dynamics(state, parameter, torques);
    // update state
    Eigen::VectorXd nexts = state + dz * p.dt;
    // compute collision, update velocity
    nexts.segment(p.dim/2, p.dim/2) = discrete_contact_dynamics_new(nexts, parameter, p.rest_coeff, p.fric_coeff, p.ground_height);
    // update position based on the corrected velocity after contact with old position
    nexts.segment(0, p.dim/2) = state.segment(0, p.dim/2) + nexts.segment(p.dim/2, p.dim/2) * p.dt;
    state = nexts;
    num_steps += 1;
    return state;
}

double Hopper2dEnv::calc_stand_reward(){
    //basic reward is 1
    double reward = 0;
    // cost from torques
    double torques_reward= -0.0001 * (torques[0] * torques[0] + torques[1] * torques[1] + torques[2] * torques[2]);
    // cost from height

    double angle_reward = exp(- (state(2) - p.init_state(2)) * (state(2) - p.init_state(2)));
    double height_reward = exp(- (state(1) - p.init_state(1)) * (state(1) - p.init_state(1)));
    double position_reward = exp(- (state(0) - p.init_state(0)) * (state(0) - p.init_state(0)));

    reward = reward + torques_reward + height_reward + angle_reward + position_reward;

    return reward;
}

double Hopper2dEnv::calc_jump_reward(){
    //basic reward is 1
    double reward = 0;
    // cost from torques
    double torques_reward= -0.0001 * (torques[0] * torques[0] + torques[1] * torques[1] + torques[2] * torques[2]);
    // cost from height

    double angle_reward = exp(- (state(2) - p.init_state(2)) * (state(2) - p.init_state(2))) * 0.01;
    double height_reward = state(1);
    double jump_reward = (state(1) > 0.7) ? state(1) : 0;
    double position_reward = exp(- (state(0) - p.init_state(0)) * (state(0) - p.init_state(0))) * 0.01;

    reward = reward + torques_reward + height_reward + angle_reward + position_reward + jump_reward * 10;

    return reward;
}

bool Hopper2dEnv::is_done(){
    if(state(1) < p.terminal_height || num_steps > p.max_steps){
        return true;
    }
    else{
        return false;
    }
}


vector<double> Hopper2dEnv::compute_torques(VectorXd actions){
    // implement pd control
    VectorXd t = VectorXd::Zero(3);

    // double amp(0.25);
    // double freq(2.5);
    // double des_pos = amp * sin(2 * M_PI * freq * num_steps * p.dt);
    // double des_vel = amp * 2 * M_PI * freq * cos(2 * M_PI * freq * num_steps * p.dt);

    // t[0] = p.kp * (p.init_state[3] + des_pos - state[3]) + p.kd * (des_vel - state[9]);
    // t[1] = p.kp * (p.init_state[4] - 2*des_pos - state[4]) + p.kd * (-2*des_vel - state[10]);
    // t[2] = p.kp * (p.init_state[5] + des_pos - state[5]) + p.kd * (des_vel - state[11]);

    t = p.kp * (p.init_state.segment(3,3) + actions - state.segment(3, 3)) - p.kd * state.segment(9, 3);
    // clip torques
    for(int i = 0; i < 3; i++){
        if(t[i] > p.max_torque[i]){
            t[i] = p.max_torque[i];
        }
        else if(t[i] < -p.max_torque[i]){
            t[i] = -p.max_torque[i];
        }
    }



    return {t[0], t[1], t[2]};
}

void Hopper2dEnv::render(){
    animator.render(state);
}
