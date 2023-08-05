#include "locooptimize2d.hpp"
#include <iostream>  
#include <typeinfo> 

loco_logdata locooptimize2d(LocoParams p, State2d s, loco_con_data cdata, MatrixXd xk_des, double desired_vel){
    int pred_hor = cdata.rcs.size();
    // store logging information
    loco_logdata log; 
    log.cd = cdata;

    DM xdk_des = DM::zeros(2,1); xdk_des(0,0) = desired_vel;
   
    //creating the optimization variables
    //1. ipopt
    Opti opti = Opti();
    //2. Gurobi
    // Opti opti;
    // opti = Opti("conic");
    Slice all;
    auto X     = opti.variable(6, pred_hor+1);
    auto x     = X(Slice(0,2), all);
    auto theta = X(Slice(2,3), all);
    auto xd    = X(Slice(3,5), all);
    auto w     = X(Slice(5,6), all);

    auto con   = opti.variable(8, pred_hor);
    auto rtoef    = con(Slice(0,2), all);
    auto rheelf    = con(Slice(2,4), all);
    auto ltoef    = con(Slice(4,6), all);
    auto lheelf    = con(Slice(6,8), all);

    //calculating the cost
    MX obj = 0;
    for (int k = 1; k < pred_hor+1; k++){
        auto xk       =  x(all, k);
        auto thetak   =  theta(all, k);
        auto xdk      =  xd(all, k);
        auto wk       =  w(all, k);
        auto rtoefk   =  rtoef(all, k-1);
        auto rheelfk  =  rheelf(all, k-1);
        auto ltoefk   =  ltoef(all, k-1);
        auto lheelfk  =  lheelf(all, k-1);
    
        auto err_xk     = xk     - EigenVectorTodm(xk_des.col(k));
        auto err_xdk    = xdk    - xdk_des;
        auto err_thetak = thetak - p.thetak_des;
        auto err_wk     = wk     - p.wk_des;

        obj += mtimes(mtimes(err_thetak.T(), p.QTheta), err_thetak)   // theta error
            +  mtimes(mtimes(    err_wk.T(), p.QW    ),     err_wk)       // w error
            +  mtimes(mtimes(    rtoefk.T(), p.QC    ),     rtoefk)          // contact force
            +  mtimes(mtimes(   rheelfk.T(), p.QC    ),     rheelfk)         // contact force
            +  mtimes(mtimes(    ltoefk.T(), p.QC    ),     ltoefk)          // contact force
            +  mtimes(mtimes(   lheelfk.T(), p.QC    ),     lheelfk)         // contact force
            +  mtimes(mtimes(    err_xk.T(), p.QX    ),     err_xk);         // x error

        if (k < pred_hor) {
            obj += mtimes(mtimes(   err_xdk.T(), p.QXd),    err_xdk);

        }else{
            obj += mtimes(mtimes(  err_xk.T(), p.QX_terminal ),    err_xk)
                 + mtimes(mtimes( err_xdk.T(), p.QXd_terminal),    err_xdk);

        }
    }
    // auto terminal_err = x(all, pred_hor) - EigenVectorTodm(xk_des.col(pred_hor));
    // auto weight = DM::eye(2) * 40;
    // obj += mtimes(mtimes( terminal_err.T(), weight),  terminal_err);

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
        auto rtoefk   =  rtoef (all, k);
        auto rheelfk  =  rheelf(all, k);
        auto ltoefk   =  ltoef (all, k);
        auto lheelfk  =  lheelf(all, k);
        // contact location for right toe location
        auto rtoeloc    =    EigenVectorTodm(cdata.rcl.col(k) + p.c_to_b);
        // contact location for point e
        auto rheeloc    =    EigenVectorTodm(cdata.rcl.col(k) + p.e_to_b);
        // contact location for right toe location  
        auto ltoeloc    =    EigenVectorTodm(cdata.lcl.col(k) + p.c_to_b);
        // contact location for point e
        auto lheeloc    =    EigenVectorTodm(cdata.lcl.col(k) + p.e_to_b);

        int rcsk    =   cdata.rcs[k];
        int lcsk   =    cdata.lcs[k];

        // dynamics constraints
        auto xdd = 1/p.M * (rtoefk + rheelfk + ltoefk + lheelfk) + p.gravity_opti;
        // ignore other parts' inertia
        auto omegaDot = 1/p.Ibody * ((rtoeloc-xk)(0)* rtoefk(1) - (rtoeloc-xk)(1)* rtoefk(0) 
                                    +(rheeloc-xk)(0)*rheelfk(1) - (rheeloc-xk)(1)*rheelfk(0)
                                    +(ltoeloc-xk)(0)* ltoefk(1) - (ltoeloc-xk)(1)* ltoefk(0) 
                                    +(lheeloc-xk)(0)*lheelfk(1) - (lheeloc-xk)(1)*lheelfk(0));

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
        // 1. right foot
        if (rcsk){
            // toe
            opti.subject_to( 0                     <= rtoefk(1) <=p.max_react_force);
            opti.subject_to(-p.fric_coeff*rtoefk(1)<= rtoefk(0) <=p.fric_coeff*rtoefk(1));
            //heel
            opti.subject_to( 0                      <= rheelfk(1) <=p.max_react_force);
            opti.subject_to(-p.fric_coeff*rheelfk(1)<= rheelfk(0) <=p.fric_coeff*rheelfk(1));
            //kinematics constraints
            auto rcl = EigenMatrixTodm(cdata.rcl.col(k));
            opti.subject_to( p.lower_bdbox  <= rcl - xk - p.fpose <= p.upper_bdbox);
        }else{
            opti.subject_to(rtoefk == DM::zeros(2,1));
            opti.subject_to(rheelfk == DM::zeros(2,1));
        }
        // 2. left foot
        if (lcsk){
            // toe
            opti.subject_to( 0                     <= ltoefk(1) <=p.max_react_force);
            opti.subject_to(-p.fric_coeff*ltoefk(1)<= ltoefk(0) <=p.fric_coeff*ltoefk(1));
            //heel
            opti.subject_to( 0                      <= lheelfk(1) <=p.max_react_force);
            opti.subject_to(-p.fric_coeff*lheelfk(1)<= lheelfk(0) <=p.fric_coeff*lheelfk(1));
            //kinematics constraints
            auto lcl = EigenMatrixTodm(cdata.lcl.col(k));
            opti.subject_to( p.lower_bdbox  <= lcl - xk - p.fpose <= p.upper_bdbox);
        }else{
            opti.subject_to(ltoefk == DM::zeros(2,1));
            opti.subject_to(lheelfk == DM::zeros(2,1));
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
    DM rtoef_init = DM::zeros(2, pred_hor);
    DM rheelf_init = DM::zeros(2, pred_hor);
    DM ltoef_init = DM::zeros(2, pred_hor);
    DM lheelf_init = DM::zeros(2, pred_hor);
    for (int i = 0; i < pred_hor; i++){
        // cout<<i<<endl;
        if (cdata.rcs[i] == 1 && cdata.lcs[i] == 1){
            rtoef_init(1, i) = rheelf_init(1, i)=ltoef_init(1, i) =lheelf_init(1, i) = -p.M * p.gravity_opti(1) / 4;
        } else if (cdata.rcs[i] == 1){
            rtoef_init(1, i) = rheelf_init(1, i) = -p.M * p.gravity_opti(1) / 2;
        } else if (cdata.lcs[i] == 1){
            ltoef_init(1, i) = lheelf_init(1, i) = -p.M * p.gravity_opti(1) / 2;
        }

    }

    

    opti.set_initial(rtoef, rtoef_init);
    opti.set_initial(rheelf, rheelf_init);
    opti.set_initial(ltoef, ltoef_init);
    opti.set_initial(lheelf, lheelf_init);

    Dict p_opts; p_opts["expand"] = true;
    p_opts["print_time"] = 0;
    Dict s_opts;      
    s_opts["max_iter"] = p.max_iter;
    s_opts["print_level"] = 0;
    s_opts["sb"] = "yes";
    opti.solver("ipopt", p_opts, s_opts);
    
    // string solverName; 
    // solverName = "gurobi"; 
    //s_opts["TimeLimit"] = 0.4;
    //s_opts["OutputFlag"] = 0;
    // opti.solver(solverName, p_opts, s_opts);
    log.done = false;
    try {
        auto sol = opti.solve();
        double c = opti.debug().value(obj).scalar();
        double average_cost = c/pred_hor;
        log.reward =  (1.0/average_cost ) * 0.1; 
        // cout<<"average_cost "<<average_cost<<endl;
        // cout<<"the cost "<< c <<endl;
        // cout<<"pred_hor "<<pred_hor<<endl;
        // cout<<"reward "<<log.reward<<endl;
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
    log.rtoef = opti.debug().value(rtoef);
    log.rheelf = opti.debug().value(rheelf);
    log.ltoef = opti.debug().value(ltoef);
    log.lheelf = opti.debug().value(lheelf);

    // cout<< "log.x \n" <<dmToEigen(log.x)<<endl;
    // cout<< "xd \n" <<dmToEigen(log.xd)<<endl;
    // cout<< "theta \n" <<dmToEigen(log.theta)<<endl;
    // cout<< "w \n" <<dmToEigen(log.w)<<endl;
    // cout<< "rtoef \n" <<dmToEigen(log.rtoef)<<endl;
    // cout<< "rheelf \n" <<dmToEigen(log.rheelf)<<endl;
    // cout<< "ltoef \n" <<dmToEigen(log.ltoef)<<endl;
    // cout<< "lheelf \n" <<dmToEigen(log.lheelf)<<endl;

    // calculate_cost(log, p, xk_des, pred_hor);

    return log;
}

// purely for debugging
void calculate_cost(loco_logdata &log, LocoParams p, MatrixXd xk_des, int pred_hor, DM xdk_des){
    double obj = 0;
    double obj_theta = 0;
    double obj_w = 0;
    double obj_x_term = 0;
    double obj_xd_term = 0;
    double obj_con = 0;
    double obj_xd = 0; 

    // calculate the cost
    for (int k = 1; k < pred_hor+1; k++){
        Vector2d err_xk     = dmToEigen(log.x).col(k) - xk_des.col(k);
        Vector2d err_xdk    = dmToEigen(log.xd).col(k) - dmToEigen(xdk_des);
        double err_thetak = dmToEigen(log.theta).col(k).value() - p.thetak_des;
        double err_wk     = dmToEigen(log.w).col(k).value() - p.wk_des;
        Vector2d rtoefk     = dmToEigen(log.rtoef).col(k-1);
        Vector2d rheelfk    = dmToEigen(log.rheelf).col(k-1);
        Vector2d ltoefk     = dmToEigen(log.ltoef).col(k-1);
        Vector2d lheelfk    = dmToEigen(log.lheelf).col(k-1);

        obj_theta += err_thetak * p.QTheta * err_thetak;
        obj_w += err_wk * p.QW * err_wk;
        obj_con += (rtoefk.transpose() * dmToEigen(p.QC) * rtoefk).value()
                + (rheelfk.transpose() * dmToEigen(p.QC) * rheelfk).value()
                + (ltoefk.transpose() * dmToEigen(p.QC) * ltoefk).value()
                + (lheelfk.transpose() * dmToEigen(p.QC) * lheelfk).value();

        if (k < pred_hor){
            obj_xd += err_xdk.transpose() * dmToEigen(p.QXd) * err_xdk;
        } 
        else{
            obj_x_term  += (err_xk.transpose() * dmToEigen(p.QX_terminal) * err_xk).value();
            obj_xd_term +=  (err_xdk.transpose() * dmToEigen(p.QXd_terminal) * err_xdk).value();

        }
    }
    obj = obj_theta + obj_w + obj_con + obj_xd + obj_x_term + obj_xd_term;
    cout<<"the total cost is "<<obj<<endl;
    cout<< "obj_theta "<<obj_theta<<endl;
    cout<< "obj_w "<<obj_w<<endl;
    cout<< "obj_con "<<obj_con<<endl;
    cout<< "obj_xd "<<obj_xd<<endl;
    cout<< "obj_x_term "<<obj_x_term<<endl;
    cout<< "obj_xd_term "<<obj_xd_term<<endl;


}
