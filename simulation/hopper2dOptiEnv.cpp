#include "hopper2dOptiEnv.hpp"


hopper2dOptiEnv::hopper2dOptiEnv(){

}

State2d hopper2dOptiEnv::step(double contact_loc, double contact_dts, double flight_dts){

    // calculate the horizon based on dts
    
    int contact_hor = floor(contact_dts / p.opt_dt);
    int flight_hor = floor(flight_dts / p.opt_dt);
    pred_hor = contact_hor + flight_hor + 1;  // +1 is for the contact after flight
    contact_data cdata = get_contact_data(contact_loc, contact_hor , flight_hor);

    MatrixXd xk_des = get_desireX();

    // cout<<"xk_des: \n"<<xk_des<<endl;
    double swing_vel = contact_loc / flight_dts;
    log = optimize2d(p, s, cdata, xk_des);
    if (log.done || num_steps >100 || swing_vel > 3){
        initstate();
        if (log.done || swing_vel > 3){
            s.reward = -10;
        }else{
            s.reward = 0;
        }
        log.done = true;
    } else {
        Slice all;
        DM x  = log.x(all,log.x.size2()-1);
        DM xd = log.xd(all,log.xd.size2()-1);
        prev_x = s.x(0);
        s.x     = dmToEigen(x);
        s.xd    = dmToEigen(xd);
        s.theta = log.theta(0,log.theta.size2()-1).scalar();
        s.w     = log.w(0,log.w.size2()-1).scalar();
        double swing_vel_penalty = exp((swing_vel -2.4) * 4.5);
        // cout<<"swing_vel_penalty: "<<swing_vel_penalty<<endl;
        double survival_reward = 0.5;
        s.reward = contact_loc* 0.5 + survival_reward + contact_hor*0.1 + flight_hor*0.1;
        // s.reward = log.reward - swing_vel_penalty + survival_reward; // reward is from the optimization
        // s.curr_contact_loc = log.cd.cl(0,log.cd.cl.cols()-1) - s.x(0); // relative location from contact to the current com
        s.curr_contact_loc = log.cd.cl(0,log.cd.cl.cols()-1);  // absolute location of the contact
    }
    num_steps++;
    
    return s;
}

State2d hopper2dOptiEnv::reset(){
    num_steps = 0;
    initstate();
    log.done = false;
    pred_hor = 0;
    prev_x = 0;
    return s;
}

// we are not using it for now
double hopper2dOptiEnv::calc_reward(){
    double reward = 0;

    double vel = (s.x(0) - prev_x) / (pred_hor * p.opt_dt);
    double vel_reward = exp(-(p.xdk_des(0,0).scalar() - vel) * (p.xdk_des(0,0).scalar() - vel) / (0.8*0.8)) / 0.8;

    reward = vel_reward;

    return reward;
}



void hopper2dOptiEnv::initstate(){
    // add a bit randomness to the state
    Vector2d x(2); x<< (double)rand()/RAND_MAX * 0.01, p.init_state(1)+(double)rand()/RAND_MAX * 0.01;
    Vector2d xd(2); xd<<(double)rand()/RAND_MAX * 0.01, (double)rand()/RAND_MAX * 0.01;
    double theta = (double)rand()/RAND_MAX * 0.01;
    double w = (double)rand()/RAND_MAX * 0.01;
    double reward = 0;
    double curr_contact_loc = (double)rand()/RAND_MAX * 0.01;
    
    s = State2d(x, xd, theta, w, reward, curr_contact_loc);
}


contact_data hopper2dOptiEnv::get_contact_data(double contact_loc, int contact_hor , int flight_hor){
    contact_data cdata;
    cdata.cl = MatrixXd::Zero(2, pred_hor);

    // push contact_hor ones into the contact data
    for (int i=0; i<contact_hor; i++){
        cdata.cs.push_back(1);
        cdata.cl(0, i) = s.curr_contact_loc;
    }

    // push flight_hor zeros into the contact data
    for (int i=0; i<flight_hor; i++){
        cdata.cs.push_back(0);
    }
    // push the 1 contact after flight
    cdata.cs.push_back(1); 
    cdata.cl(0, pred_hor-1) = s.curr_contact_loc + contact_loc;
    // cdata.cl(0, pred_hor-1) = s.x(0) + contact_loc;

    return cdata;
}


MatrixXd hopper2dOptiEnv::get_desireX(){
    MatrixXd xk_des = MatrixXd::Zero(2, pred_hor+1);

    for (int i = 0; i < pred_hor+1; i++){
        // current we just want to maintain the same height as the initial state, may be it's not ideal for a hopping robot. 
        xk_des(1, i) =  p.init_state(1);
    }
    xk_des(0, pred_hor) = 0;
    return xk_des;
}




void hopper2dOptiEnv::render(){
    animator.animate(log);
}




