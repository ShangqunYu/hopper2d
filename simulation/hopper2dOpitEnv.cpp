#include "hopper2dOpitEnv.hpp"


hopper2dOpitEnv::hopper2dOpitEnv(){

}

State2d hopper2dOpitEnv::step(double contact_loc, double contact_dts, double flight_dts){

    // calculate the horizon based on dts
    
    int contact_hor = floor(contact_dts / p.opt_dt);
    int flight_hor = floor(flight_dts / p.opt_dt);
    pred_hor = contact_hor + flight_hor + 1;  // +1 is for the contact after flight

    contact_data cdata = get_contact_data(contact_loc, contact_hor , flight_hor);

    MatrixXd xk_des = get_desireX();

    // cout<<"xk_des: \n"<<xk_des<<endl;
    log = optimize2d(p, s, cdata, xk_des);

    if (log.done || num_steps >100 || s.x(0)>p.maxX){
        initstate();
        if (log.done){
            s.reward = -10;
        }else{
            s.reward = 0;
            log.done = true;
        }
    } else {
        Slice all;
        DM x  = log.x(all,log.x.size2()-1);
        DM xd = log.xd(all,log.xd.size2()-1);
        s.x     = dmToEigen(x);
        s.xd    = dmToEigen(xd);
        s.theta = log.theta(0,log.theta.size2()-1).scalar();
        s.w     = log.w(0,log.w.size2()-1).scalar();
        s.reward = log.reward; // reward is from the optimization
    }
    num_steps++;
    
    return s;
}

void hopper2dOpitEnv::reset(){
    num_steps = 0;
    initstate();
    log.done = false;
    generate_terrain();
}

VectorXd hopper2dOpitEnv::get_terrian_obs(){
    double x = (s.x(0)<0)?0:s.x(0);
    int idx = (int)floor(x/0.25);
    VectorXd obs(p.terrian_hor);
    for (int i=0; i<p.terrian_hor; i++){
        obs(i) = terrain[idx+1+i];
    }
    return obs;
}



void hopper2dOpitEnv::initstate(){
    Vector2d x(2); x<< 0, p.h;
    Vector2d xd(2); xd<< 0, 0;
    double theta = 0;
    double w = 0;
    double reward = 0;
    s = State2d(x, xd, theta, w, reward);
}


contact_data hopper2dOpitEnv::get_contact_data(contact_loc, contact_hor , flight_hor){
    int pred_hor = contact_hor + flight_hor + 1;  // +1 is for the contact after flight
    contact_data cdata;
    cdata.rcl = MatrixXd::Zero(2, pred_hor);
    cdata.lcl = MatrixXd::Zero(2, pred_hor);

    // push contact_hor ones into the contact data
    for (int i=0; i<contact_hor; i++){
        cdata.cs.push_back(1);
        cdata.rcl(0, i) = s.x(0) + cur_contact_loc + p.l2;
        cdata.lcl(0, i) = s.x(0) + cur_contact_loc - p.l21;
    }

    // push flight_hor ones into the contact data
    for (int i=0; i<flight_hor; i++){
        cdata.cs.push_back(0);
    }
    // push the 1 contact after flight
    cdata.cs.push_back(1); 
    cdata.rcl(0, contact_hor) = s.x(0) + contact_loc + p.l2;
    cdata.lcl(0, contact_hor) = s.x(0) + contact_loc - p.l21;

    return cdata;
}

bool hopper2dOpitEnv::checkinthepit(Vector2d rxs, Vector2d lxs){
    for (int i=0; i<rxs.size(); i++){
        double x = rxs(i)+s.x(0);
        x = (x<0)?0:x;
        int idx = (int)floor(x/0.25);
        if (terrain(idx)==0){
            return true;
        }
    }
    for (int i=0; i<lxs.size(); i++){
        double x = lxs(i)+s.x(0);
        x = (x<0)?0:x;
        int idx = (int)floor(x/0.25);
        if (terrain(idx)==0){
            return true;
        }
    }
    return false;
}

MatrixXd hopper2dOpitEnv::get_desireX(){
    MatrixXd xk_des = MatrixXd::Zero(2, p.pred_hor+1);

    for (int i = 0; i < p.pred_hor+1; i++){
        xk_des(1, i) =  p.init_state(1);

    }
    return xk_des;
}

contact_data hopper2dOpitEnv::get_cs(int pred_hor, double dt, VectorXd dts, bool is_right){
    contact_data out;
    double sumVal = dts.sum();
    int allocated[dts.size()];
    for (int i = 0; i < dts.size(); i++){
        allocated[i] = round (dts(i)/sumVal * pred_hor);
    }
    // hacky way to make sure the sum is equal to pred_hor
    allocated[0] += pred_hor - allocated[0] - allocated[1] - allocated[2] - allocated[3];

    for (int i = 0; i < dts.size(); i++){
        for (int j = 0; j < allocated[i]; j++)
        {
            out.cindex.push_back(i);
            if (is_right){
                out.cs.push_back(i % 2 == 0);
            }else{
                out.cs.push_back(i % 2 == 1);
            }
        }
        
    }
    return out;
}



void hopper2dOpitEnv::render(){
    animator.animate(log, terrain);
}




