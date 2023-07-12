#include "loco2dOptiEnv.hpp"


loco2dOptiEnv::loco2dOptiEnv(){

}
                            
State2d loco2dOptiEnv::step(double r_contact_loc, double r_contact_dts, double r_flight_dts, double l_contact_loc, double l_flight_dts, double l_contact_dts){

    // calculate the horizon based on dts
    
    int r_contact_hor = floor(r_contact_dts / p.opt_dt);
    int r_flight_hor = floor(r_flight_dts / p.opt_dt);
    pred_hor = r_contact_hor + r_flight_hor + r_contact_hor;  // +1 is for the contact after flight

    int l_flight_hor = floor(l_flight_dts / p.opt_dt);

    int l_contact_hor = floor(l_contact_dts / p.opt_dt);

    // cout<<"pred_hor: "<<pred_hor<<endl;
    // cout<<"r_contact_hor: "<<r_contact_hor<<endl;
    // cout<<"r_flight_hor: "<<r_flight_hor<<endl;
    // cout<<"l_flight_hor: "<<l_flight_hor<<endl;
    // cout<<"l_contact_hor: "<<l_contact_hor<<endl;

    loco_con_data cdata = get_contact_data(r_contact_loc, r_contact_hor , r_flight_hor, l_contact_loc, l_flight_hor, l_contact_hor);

    MatrixXd xk_des = get_desireX();
    // cout<<"xk_des: "<<xk_des<<endl;
    log = locooptimize2d(p, s, cdata, xk_des);
    // cout<<"log.x: "<<log.x<<endl;
    if (log.done || num_steps >100){
        initstate();
        if (log.done){
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

        double survival_reward = 0.5;
        // s.reward = contact_loc* 0.5 + survival_reward + contact_hor*0.1 + flight_hor*0.1;
        s.reward = log.reward  + survival_reward; // reward is from the optimization
        // s.curr_contact_loc = log.cd.cl(0,log.cd.cl.cols()-1) - s.x(0); // relative location from contact to the current com
        s.curr_contact_loc = log.cd.rcl(0,log.cd.rcl.cols()-1);  // absolute location of the contact
    }
    num_steps++;
    
    return s;
}

State2d loco2dOptiEnv::reset(){
    num_steps = 0;
    initstate();
    log.done = false;
    pred_hor = 0;
    prev_x = 0;
    return s;
}

// we are not using it for now
double loco2dOptiEnv::calc_reward(){
    double reward = 0;

    double vel = (s.x(0) - prev_x) / (pred_hor * p.opt_dt);
    double vel_reward = exp(-(p.xdk_des(0,0).scalar() - vel) * (p.xdk_des(0,0).scalar() - vel) / (0.8*0.8)) / 0.8;

    reward = vel_reward;

    return reward;
}



void loco2dOptiEnv::initstate(){
    // add a bit randomness to the state
    Vector2d x(2); x<< (double)rand()/RAND_MAX * 0.01, p.init_state(1)+(double)rand()/RAND_MAX * 0.01;
    Vector2d xd(2); xd<<(double)rand()/RAND_MAX * 0.01, (double)rand()/RAND_MAX * 0.01;
    double theta = (double)rand()/RAND_MAX * 0.01;
    double w = (double)rand()/RAND_MAX * 0.01;
    double reward = 0;
    double curr_contact_loc = (double)rand()/RAND_MAX * 0.01;
    
    s = State2d(x, xd, theta, w, reward, curr_contact_loc);
}


loco_con_data loco2dOptiEnv::get_contact_data(double r_contact_loc, int r_contact_hor, int r_flight_hor, double l_contact_loc, int l_flight_hor, int l_contact_hor){
    loco_con_data cdata;
    cdata.rcl = MatrixXd::Zero(2, pred_hor);
    cdata.lcl = MatrixXd::Zero(2, pred_hor);

    // working on the right foot first
    // push contact_hor ones into the contact data
    for (int i=0; i<r_contact_hor; i++){
        cdata.rcs.push_back(1);
        cdata.rcl(0, i) = s.curr_contact_loc;
    }

    // push flight_hor zeros into the contact data
    for (int i=0; i<r_flight_hor; i++){
        cdata.rcs.push_back(0);
    }

    // push contact_hor ones into the contact data
    for (int i=0; i<r_contact_hor; i++){
        cdata.rcs.push_back(1);
        cdata.rcl(0, i+r_contact_hor+r_flight_hor) = s.curr_contact_loc + r_contact_loc;
    }

    //output size of cdata.rcl
    // cout<<"cdata.rcl.cols(): "<<cdata.rcl.cols()<<endl;

    // working onf the left foot
    // push l_flight_hor zeros into the contact data
    for (int i=0; i<l_flight_hor; i++){
        cdata.lcs.push_back(0);
    }
    // prevent l_contact_hor + l_flight_hor > pred_hor
    l_contact_hor = min(l_contact_hor, pred_hor - l_flight_hor);

    // push adjusted l_contact_hor ones into the contact 
    for (int i=0; i<l_contact_hor; i++){
        cdata.lcs.push_back(1);
        cdata.lcl(0, i+l_flight_hor) = s.curr_contact_loc + l_contact_loc;
    }

    for (int i=0; i<pred_hor-l_flight_hor-l_contact_hor; i++){
        cdata.lcs.push_back(0);
    }

    // cout<< "rcl \n"<< cdata.rcl<<endl;
    // cout<< "lcl \n"<< cdata.lcl<<endl;

    return cdata;
}


MatrixXd loco2dOptiEnv::get_desireX(){
    MatrixXd xk_des = MatrixXd::Zero(2, pred_hor+1);

    for (int i = 0; i < pred_hor+1; i++){
        // current we just want to maintain the same height as the initial state. 
        xk_des(1, i) =  p.init_state(1);
    }
    double dist = pred_hor * p.opt_dt * 1.0;
    xk_des(0, pred_hor) = s.x(0) +  max(p.min_dist, dist );
    xk_des(1, pred_hor) = p.init_state(1);
    // cout<<"xk_des(1, pred_hor) "<< xk_des(1, pred_hor)<<endl;
    return xk_des;
}




void loco2dOptiEnv::render(){
    animator.animate_loco(log);
}



