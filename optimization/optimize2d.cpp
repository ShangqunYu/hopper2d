#include "optimize2d.hpp"
#include <iostream>  
#include <typeinfo> 

logdata optimize2d(Parameters p, State2d s, contact_data cdata, MatrixXd xk_des){
    int pred_hor = cdata.cs.size();
    // store logging information
    logdata log; 
    log.cd = cdata;
   
    //creating the optimization variables
    Opti opti = Opti();
    Slice all;
    auto X     = opti.variable(6, pred_hor+1);
    auto x     = X(Slice(0,2), all);
    auto theta = X(Slice(2,3), all);
    auto xd    = X(Slice(3,5), all);
    auto w     = X(Slice(5,6), all);

    auto con   = opti.variable(4, pred_hor);
    auto cf    = con(Slice(0,2), all);
    auto ef    = con(Slice(2,4), all);

    //calculating the cost
    MX obj = 0;
    for (int k = 1; k < pred_hor+1; k++){
        auto xk     =     x(all, k);
        auto thetak = theta(all, k);
        auto xdk    =    xd(all, k);
        auto wk     =     w(all, k);
        auto cfk    =  cf(all, k-1);
        auto efk    =  ef(all, k-1);

        auto err_xk     = xk     - EigenVectorTodm(xk_des.col(k));
        auto err_xdk    = xdk    - p.xdk_des;
        auto err_thetak = thetak - p.thetak_des;
        auto err_wk     = wk     - p.wk_des;

        obj += mtimes(mtimes(    err_xk.T(), p.QX),     err_xk)
             + mtimes(mtimes(   err_xdk.T(), p.QXd),    err_xdk)
             + mtimes(mtimes(err_thetak.T(), p.QTheta), err_thetak)
             + mtimes(mtimes(    err_wk.T(), p.QW),     err_wk)
             + mtimes(mtimes(       cfk.T(), p.QC),     cfk)
             + mtimes(mtimes(       efk.T(), p.QC),     efk);
    }
    opti.minimize(obj);

    // initial constraint
    opti.subject_to(x(all, 0)     == EigenVectorTodm(s.x));
    opti.subject_to(theta(all, 0) == DM(s.theta));
    opti.subject_to(xd(all, 0)    == EigenVectorTodm(s.xd));
    opti.subject_to(w(all, 0)     == DM(s.w));

    // constraints
    for (int k = 0; k < pred_hor; k++){
        auto xk     =     x(all, k);
        auto thetak = theta(all, k);
        auto xdk    =    xd(all, k);
        auto wk     =     w(all, k);
        auto cfk    =    cf(all, k);
        auto efk    =    ef(all, k);
        // contact location for point c
        auto rck    =    EigenVectorTodm(cdata.cl.col(k) + p.c_to_b);
        // contact location for point e
        auto rek    =    EigenVectorTodm(cdata.cl.col(k) + p.e_to_b);

        int csk    =    cdata.cs[k];

        // dynamics constraints
        auto xdd = 1/p.M * (cfk + efk) + p.gravity_opti;
        // ignore other parts' inertia
        auto omegaDot = 1/p.Ibody * ((rck-xk)(0)*cfk(1) - (rck-xk)(1)*cfk(0) 
                                    +  (rek-xk)(0)*efk(1) - (rek-xk)(1)*efk(0));

        // intergrate the dynamics
        // 1. velocity
        opti.subject_to(xd(all, k+1) - xdk == xdd*p.opt_dt);
        // 2. position
        opti.subject_to(x(all, k+1) - xk   == xdk*p.opt_dt);
        // 3. angular velocity
        opti.subject_to(w(all, k+1) - wk   == omegaDot*p.opt_dt);
        // 4. angle
        opti.subject_to(theta(all, k+1) - thetak == wk*p.opt_dt);

        // contact constraints && friction constraints
        // 1. right & left foot
        if (csk){
            // right foot
            opti.subject_to( 0                  <= cfk(1) <=p.max_react_force);
            opti.subject_to(-p.fric_coeff*cfk(1)<= cfk(0) <=p.fric_coeff*cfk(1));
            //left foot
            opti.subject_to( 0                  <= efk(1) <=p.max_react_force);
            opti.subject_to(-p.fric_coeff*efk(1)<= efk(0) <=p.fric_coeff*efk(1));
            //kinematics constraints
            auto cl = EigenMatrixTodm(cdata.cl.col(k));
            opti.subject_to( p.lower_bdbox  <= cl - xk - p.fpose <= p.upper_bdbox);
        }else{
            opti.subject_to(cfk == DM::zeros(2,1));
            opti.subject_to(efk == DM::zeros(2,1));
        }

        // constraint for the theta
        opti.subject_to(-p.theta_max <= theta(all, k+1) <= p.theta_max);

    }

    // set the initial guess
    // 1. position
    opti.set_initial(x,     DM::repmat(EigenVectorTodm(s.x),  1, pred_hor+1));
    // 2. angle
    opti.set_initial(theta, DM::repmat(DM(s.theta),           1, pred_hor+1));
    // 3. velocity
    opti.set_initial(xd,    DM::repmat(EigenVectorTodm(s.xd), 1, pred_hor+1));
    // 4. angular velocity
    opti.set_initial(w,     DM::repmat(DM(s.w),               1, pred_hor+1));

    // set the initial guess for the reaction force
    DM cf_init = DM::zeros(2, pred_hor);
    DM ef_init = DM::zeros(2, pred_hor);
    for (int i = 0; i < pred_hor; i++){
        if (cdata.cs[i]){
            cf_init(1, i) = ef_init(1, i)= -p.M * p.gravity_opti(1) / 2;
        }
    }

    opti.set_initial(cf, cf_init);
    opti.set_initial(ef, ef_init);

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
        double average_cost = c/pred_hor;
        // cout<<"average_cost "<<average_cost<<endl;
        log.reward =  (1.0/average_cost - 1.0/35.0) * 35.0; 
        // cout<< "the cost "<< c <<endl;
        // cout<<"pred_hor "<<pred_hor<<endl;
        // cout<<"reward "<<log.reward<<endl;
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
    log.cf = opti.debug().value(cf);
    log.ef = opti.debug().value(ef);

    // cout<< "log.x \n" <<dmToEigen(log.x)<<endl;
    // cout<< "xd \n" <<dmToEigen(log.xd)<<endl;
    // cout<< "theta \n" <<dmToEigen(log.theta)<<endl;
    // cout<< "w \n" <<dmToEigen(log.w)<<endl;
    // cout<< "cf \n" <<dmToEigen(log.cf)<<endl;
    // cout<< "ef \n" <<dmToEigen(log.ef)<<endl;
    // cout<< "contact loc \n" << cdata.cl<<   endl;

    return log;
}
