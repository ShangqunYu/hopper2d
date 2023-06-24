#include "optimize2d.hpp"
#include <iostream>  
#include <typeinfo> 

logdata optimize2d(Parameters2d p, State2d s, contact_data cdata, MatrixXd xk_des){
    int pred_hor = cdata.cs.size();
    // store logging information
    logdata log; 
    log.rc = MatrixXd::Zero(2, pred_hor);
    log.lc = MatrixXd::Zero(2, pred_hor);
   
    //creating the optimization variables
    Opti opti = Opti();
    Slice all;
    auto X     = opti.variable(6, pred_hor+1);
    auto x     = X(Slice(0,2), all);
    auto theta = X(Slice(2,3), all);
    auto xd    = X(Slice(3,5), all);
    auto w     = X(Slice(5,6), all);

    auto con   = opti.variable(4, pred_hor);
    auto rf    = con(Slice(0,2), all);
    auto lf    = con(Slice(2,4), all);

    //calculating the cost
    MX obj = 0;
    for (int k = 1; k < p.pred_hor+1; k++){
        auto xk     =     x(all, k);
        auto thetak = theta(all, k);
        auto xdk    =    xd(all, k);
        auto wk     =     w(all, k);
        auto rfk    =  rf(all, k-1);
        auto lfk    =  lf(all, k-1);

        auto err_xk     = xk     - EigenVectorTodm(xk_des.col(k));
        auto err_xdk    = xdk    - p.xdk_des;
        auto err_thetak = thetak - p.thetak_des;
        auto err_wk     = wk     - p.wk_des;

        obj += mtimes(mtimes(    err_xk.T(), p.QX),     err_xk)
             + mtimes(mtimes(   err_xdk.T(), p.QXd),    err_xdk)
             + mtimes(mtimes(err_thetak.T(), p.QTheta), err_thetak)
             + mtimes(mtimes(    err_wk.T(), p.QW),     err_wk)
             + mtimes(mtimes(       rfk.T(), p.QC),     rfk)
             + mtimes(mtimes(       lfk.T(), p.QC),     lfk);
    }
    opti.minimize(obj);

    // initial constraint
    opti.subject_to(x(all, 0)     == EigenVectorTodm(s.x));
    opti.subject_to(theta(all, 0) == DM(s.theta));
    opti.subject_to(xd(all, 0)    == EigenVectorTodm(s.xd));
    opti.subject_to(w(all, 0)     == DM(s.w));

    // constraints
    for (int k = 0; k < p.pred_hor; k++){
        auto xk     =     x(all, k);
        auto thetak = theta(all, k);
        auto xdk    =    xd(all, k);
        auto wk     =     w(all, k);
        auto rfk    =    rf(all, k);
        auto lfk    =    lf(all, k);
        auto rck    =    EigenVectorTodm(cdata.rcl.col(k));
        auto lck    =    EigenVectorTodm(cdata.lcl.col(k));

        int csk    =    cdata.cs[k];

        // dynamics constraints
        auto xdd = 1/p.m * (rfk + lfk) + EigenMatrixTodm(p.gravity);
        auto omegaDot = 1/p.inertia * ((rck-xk)(0)*rfk(1) - (rck-xk)(1)*rfk(0) 
                                    +  (lck-xk)(0)*lfk(1) - (lck-xk)(1)*lfk(0));

        // intergrate the dynamics
        // 1. velocity
        opti.subject_to(xd(all, k+1) - xdk == xdd*p.dt);
        // 2. position
        opti.subject_to(x(all, k+1) - xk   == xdk*p.dt);
        // 3. angular velocity
        opti.subject_to(w(all, k+1) - wk   == omegaDot*p.dt);
        // 4. angle
        opti.subject_to(theta(all, k+1) - thetak == wk*p.dt);

        // contact constraints && friction constraints
        // 1. right & left foot
        if (csk){
            // right foot
            opti.subject_to(  0          <=     rfk(1)     <=p.max_react_force);
            opti.subject_to( -p.mu*rfk(1)<=     rfk(0)     <=p.mu*rfk(1));
            //left foot
            opti.subject_to(  0          <=     lfk(1)     <=p.max_react_force);
            opti.subject_to( -p.mu*lfk(1)<=     lfk(0)     <=p.mu*lfk(1));
            //kinematics constraints
            opti.subject_to( p.lower_bdbox    <= (rck +lck)/2 -xk-p.fpose <= p.upper_bdbox);
        }else{
            opti.subject_to(rfk == DM::zeros(2,1));
            opti.subject_to(lfk == DM::zeros(2,1));
        }


        // constraint for the theta
        opti.subject_to(-p.theta_max <= theta(all, k+1) <= p.theta_max);

        // logging store the contact location for each step. 
        log.rc.col(k) = rcl.col(rout.cindex[k]);
        log.lc.col(k) = lcl.col(lout.cindex[k]);

    }

    // set the initial guess
    // 1. position
    opti.set_initial(x,     DM::repmat(EigenVectorTodm(s.x),  1, p.pred_hor+1));
    // 2. angle
    opti.set_initial(theta, DM::repmat(DM(s.theta),           1, p.pred_hor+1));
    // 3. velocity
    opti.set_initial(xd,    DM::repmat(EigenVectorTodm(s.xd), 1, p.pred_hor+1));
    // 4. angular velocity
    opti.set_initial(w,     DM::repmat(DM(s.w),               1, p.pred_hor+1));

    // set the initial guess for the contact
    DM rf_init = DM::zeros(2, p.pred_hor);
    DM lf_init = DM::zeros(2, p.pred_hor);
    for (int i = 0; i < p.pred_hor; i++){
        if (rout.cs[i]){
            rf_init(1, i) = -p.m * p.gravity(1) / (rout.cs[i] + lout.cs[i]);
        }
        if (lout.cs[i]){
            lf_init(1, i) = -p.m * p.gravity(1) / (rout.cs[i] + lout.cs[i]);
        }
    }

    opti.set_initial(rf, rf_init);
    opti.set_initial(lf, lf_init);

    Dict p_opts; p_opts["expand"] = true;
    p_opts["print_time"] = 0;
    Dict s_opts; s_opts["max_iter"] = p.max_iter;
    s_opts["print_level"] = 0;
    s_opts["sb"] = "yes";
    
    opti.solver("ipopt", p_opts, s_opts);
    log.done = false;
    try {
        auto sol = opti.solve();
        double c = opti.debug().value(obj).scalar();
        log.reward = 10000 * (1.0/c - 1.0 / 1000);
        //cout<< "the cost "<< c <<endl;
        //cout<< "the reward "<< log.reward <<endl;
    }
    catch (CasadiException) {
    // Block of code to handle errors
        //cout<< "the result is infeassible"<<endl;
        log.done = true;
        log.reward = -10;
    }

    
    log.x  = opti.debug().value(x);
    log.xd = opti.debug().value(xd);
    log.w  = opti.debug().value(w);
    log.theta = opti.debug().value(theta);
    log.rf = opti.debug().value(rf);
    log.lf = opti.debug().value(lf);
    log.rcs = rout;
    log.lcs = lout;
    return log;
}
