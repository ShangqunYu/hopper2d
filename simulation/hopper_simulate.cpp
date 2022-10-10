#include "hopper_simulate.hpp"

using namespace casadi;
using namespace std;
void hopper_simulate(){

    //HYPER PARAMETER
    // WEIGHT FOR THE CONTROL
    int WEIGHT_DY = 1000; int WEIGHT_DTH=200000; int WEIGHT_TH = 10000; int WEIGHT_DX = 1000;
    double WEIGHT_CONTROL = 0.1; double WEIGHT_TERMINAL = 100;
    // TARGET STATE
    double TARGET_X_VELOCITX = 0; double TARGET_Y_VELOCITX = 8;
    double REACTION_FORCE_CONSTRAINT_Y = 350; double REACTION_FORCE_CONSTRAINT_X = 100;
    //robot property
    double mbody = 4; double m0 = 0.3; double m1 = 0.5; double m2 = 0.1;  double M = mbody + m0 + m1 + m2; // Mass
    double Ibody = 11 * pow(10,-3); double I0 = 5.1 * pow(10,-6); double I1 = 5.1 * pow(10,-6); double I2 = 1.5 * pow(10,-6); // inertia
    double l0 = 0.2; double l1 = 0.22; double l2 = 0.1;  double l21 = 0.06; double lbody = 0.3; // length of link
    double c0 = 0.1; double c1 = 0.1; double c2 = 0.07; // length to center of mass
    double gravity = 9.81; 
    double ground_height = 0;
    double rest_coeff = 0.2; 
    double fric_coeff = 0.7;

    double tf = 0.605;
    double sim_dt = 0.001;
    int num_sim_step = floor(tf/sim_dt);
    int dim = 12;
    vector<double> parameter = {mbody, Ibody, m0, m1, m2, I0, I1, I2, c0, c1, c2, l0, l1, l2, l21, gravity, lbody};
    Eigen::VectorXd z0(dim); z0<< 0.0, 0.4, 0.0, M_PI/6, -M_PI/3, M_PI/6, 0.0, 0.0,  0.0,  0.0,  0.0, 0.0; 
    z0(1) = l0*cos(z0(3)) + l1*cos(z0(3)+z0(4)) + ground_height + lbody/2;
    Eigen::MatrixXd z_out(dim,num_sim_step);
    vector<double> taus = {0,0,0};
    z_out.block(0,0,dim,1) = z0;

    // start making casadi stuff
    int N = 40;  // prediction
    double dt = 0.005; //delta t for trajectory optimization
    int n_state = 6; int n_control = 3;
    //reference state
    casadi::DM xs = casadi::DM::zeros(n_state); xs(3) = TARGET_X_VELOCITX; xs(4) = TARGET_Y_VELOCITX;
    casadi::SX X = casadi::SX::sym("X", n_state, (N+1));
    casadi::SX Fx = casadi::SX::sym("Fx",N);
    casadi::SX Fy = casadi::SX::sym("Fy", N);
    casadi::SX Tau = casadi::SX::sym("Tau", N);
    casadi::SX Q = casadi::SX::zeros(n_state,n_state);
    Q(2,2) = WEIGHT_TH; Q(3,3) = WEIGHT_DX; Q(4,4) = WEIGHT_DY; Q(5,5) = WEIGHT_DTH;
    casadi::SX R = casadi::SX::eye(n_control) * WEIGHT_CONTROL;
    
    casadi::SX P = casadi::SX::sym("P", n_state+n_state);
    casadi::SX obj = 0;
    casadi::SX init_state = X(casadi::Slice(0,n_state), 0);
    casadi::SX desired_state = P(casadi::Slice(n_state, n_state*2));
    cout<<desired_state.size()<<endl;
    casadi::SX g = init_state - P(casadi::Slice(0,n_state));
    for (int i = 0; i < N; i++){
        casadi::SX st = X(casadi::Slice(0,n_state), i);
        casadi::SX fx = Fx(i); casadi::SX fy = Fy(i); casadi::SX t = Tau(i); 
        casadi::SX con = vertcat(fx, fy, t);
        
        casadi::SX state_cost = casadi::SX::mtimes((st - desired_state).T(), casadi::SX::mtimes(Q, st - desired_state));
        casadi::SX control_cost = casadi::SX::mtimes( con.T(), casadi::SX::mtimes(R, con));
        // if it's terminal state, get more weight
        if (i ==N - 1){
            state_cost = state_cost * WEIGHT_TERMINAL;
        }
        obj = obj +  state_cost + control_cost;
    
        casadi::SX st_next = X(casadi::Slice(0,n_state), i+1);
        casadi::SX ds= centroidal_dynamics(st, fx, fy, t, M, Ibody, gravity, ground_height);
        //cout<< ds <<endl;
        casadi::SX st_next_euler = st + (dt * ds);
        g = vertcat(g, st_next - st_next_euler);
    }
    // constraint for Fz1 and Fz2
    for (int i =  0; i < N; i ++){
        casadi::SX fy = Fy(i); casadi::SX t = Tau(i);
        casadi::SX fz1 = (fy*l2 - t)/(l2+l21); casadi::SX fz2 =  (t + fy*l21) / (l2 + l21);
        g = vertcat(g, fz1, fz2);
    }
    // constraint for friction
    for (int i =  0; i < N; i ++){
        casadi::SX fx = Fx(i); casadi::SX fy = Fy(i);
        //-f_y*u<f_x            f_x< f_y*u
        g = vertcat(g, -fy*fric_coeff-fx, -fy*fric_coeff+fx);
    }
    // constraint for leg length
    for (int i =  0; i < N + 1; i ++){
        casadi::SX x = X(0,i);  casadi::SX y = X(1,i); casadi::SX theta = X(2,i); 
        casadi::SX ro_x = x + lbody / 2 * sin(theta);
        casadi::SX ro_y = y - lbody / 2 * cos(theta);
        casadi::SX dist = sqrt(pow(ro_x - 0, 2) + pow(ro_y - ground_height, 2));
        g = vertcat(g, dist);
    }
    casadi::SX OPT_variables = casadi::SX::reshape(X, n_state*(N+1), 1);
    OPT_variables = vertcat(OPT_variables , Fx);
    OPT_variables = vertcat(OPT_variables , Fy);
    OPT_variables = vertcat(OPT_variables , Tau);
    Dict opts = {{"ipopt.tol", 1e-12}, {"ipopt.max_iter", 200000}};

    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}, {"p", P}};
    Function solver = nlpsol("solver", "ipopt", nlp, opts);
    std::map<std::string, DM> arg, res;
    // equality constraint for the model
    arg["lbg"] = casadi::DM::zeros(n_state*(N+1));
    arg["ubg"] = casadi::DM::zeros(n_state*(N+1));

    // inequality constrints for fz1 fz2
    arg["lbg"] = vertcat(arg["lbg"], casadi::DM::zeros(2*N));
    arg["ubg"] =vertcat(arg["ubg"], casadi::DM::ones(2*N) * 500000);

    // inequality constraints for friction lower bound
    arg["lbg"] = vertcat(arg["lbg"], casadi::DM::ones(2*N) * -500000);
    arg["ubg"] =vertcat(arg["ubg"], casadi::DM::zeros(2*N)); 

    // constraints for leg length
    arg["lbg"] = vertcat(arg["lbg"], casadi::DM::zeros(N+1));
    arg["ubg"] =vertcat(arg["ubg"], casadi::DM::ones(N+1) * (l0+l1));
    cout<<"LBG Size:" << arg["lbg"].size() <<endl;
    //opt_variable constraints state
    for (int i = 0; i< N+1; i++){
        // state x bound
        arg["lbx"] = vertcat(arg["lbx"], -5); arg["ubx"] = vertcat(arg["ubx"], 5);
        // state y bound
        arg["lbx"] = vertcat(arg["lbx"], 0); arg["ubx"] = vertcat(arg["ubx"], 20);
        // state th bound
        arg["lbx"] = vertcat(arg["lbx"], -5000); arg["ubx"] = vertcat(arg["ubx"], 5000);
        // state dx bound
        arg["lbx"] = vertcat(arg["lbx"], -10); arg["ubx"] = vertcat(arg["ubx"], 10);
        // state dy bound
        arg["lbx"] = vertcat(arg["lbx"], -100); arg["ubx"] = vertcat(arg["ubx"], 100);
        // state dth bound
        arg["lbx"] = vertcat(arg["lbx"], -100); arg["ubx"] = vertcat(arg["ubx"], 100);
    }
    cout<<arg["lbx"].size()<<endl;
    //opt_variable constraints control
    //constraint for Fx
    arg["lbx"] = vertcat(arg["lbx"], casadi::DM::ones(N) * -REACTION_FORCE_CONSTRAINT_X );
    arg["ubx"] = vertcat(arg["ubx"], casadi::DM::ones(N) *  REACTION_FORCE_CONSTRAINT_X );
    //constraint for Fy
    arg["lbx"] = vertcat(arg["lbx"], casadi::DM::ones(N) *  0 );
    arg["ubx"] = vertcat(arg["ubx"], casadi::DM::ones(N) *  REACTION_FORCE_CONSTRAINT_Y );
    
    //constraint for overall torque
    arg["lbx"] = vertcat(arg["lbx"], casadi::DM::ones(N) *  -40 );
    arg["ubx"] = vertcat(arg["ubx"], casadi::DM::ones(N) *  40 );
    cout<<arg["lbx"].size()<<endl;

    //setting up the parameter
    casadi::DM x0 = casadi::DM::zeros(n_state); x0(0) = z0(0); x0(1) = z0(1); x0(2) = z0(2);
    arg["p"] = vertcat(x0, xs);;
    //setting up initial guess
    casadi::DM X0 = casadi::DM::repmat(x0,1, N+1);
    casadi::DM fx0 = casadi::DM::zeros(N);
    casadi::DM fy0 = casadi::DM::zeros(N);
    casadi::DM tau0 = casadi::DM::zeros(N);
    arg["x0"] = casadi::SX::reshape(X0, n_state*(N+1), 1);
    arg["x0"] = vertcat(arg["x0"], fx0, fy0);
    arg["x0"] = vertcat(arg["x0"], tau0);
    //ask casadi to solve the problem
    res = solver(arg);
    //get the results.
    vector<double> fxs = res.at("x")(casadi::Slice(n_state*(N+1), n_state*(N+1) + N)).get_elements();
    vector<double> fys = res.at("x")(casadi::Slice(n_state*(N+1)+N, n_state*(N+1) + 2*N)).get_elements();
    vector<double> torques = res.at("x")(casadi::Slice(n_state*(N+1)+2*N, n_state*(N+1) + 3*N)).get_elements();

    double opt_horizon = N * dt; 
    double sim_time = 0;
    Eigen::VectorXd jpos_jump = z0;
    bool Left_ground = false;
    double* J; J = new double [18];
    for(int i = 0; i < num_sim_step-1; i++){
        sim_time = sim_time + sim_dt;
        Eigen::MatrixXd cur_z = z_out.block(0,i,dim,1);
        if (sim_time < opt_horizon){
            int input_idx = floor(sim_time/opt_horizon*N);
            Eigen::Vector3d outmodel(-fxs[input_idx],-fys[input_idx],-torques[input_idx]);
            vector<double> curr_z  (cur_z.data(), cur_z.data() + cur_z.rows()*cur_z.cols());
            jacobian_b(&curr_z[0], &parameter[0], J);
            Eigen::MatrixXd J_matrix = arrayToEigen(J, n_control, n_state);
            Eigen::VectorXd tau =  J_matrix.transpose() * outmodel;
            vector<double> tau_(tau.data(), tau.data() + tau.rows() * tau.cols());
            
            taus = {tau_[3], tau_[4], tau_[5]};
            
        }
        else{
            if (!Left_ground){
                jpos_jump = cur_z;
                Left_ground = true;
            }
            taus =  basic_control(jpos_jump, cur_z);
        }
        
        Eigen::VectorXd dz = dynamics(cur_z, parameter, taus);
        z_out.block(0,i+1,dim,1) = cur_z + dz * sim_dt;
        z_out.block(dim/2,i+1,dim/2,1)= discrete_contact_dynamics(z_out.block(0,i+1,dim,1), parameter, rest_coeff, fric_coeff, ground_height);
        z_out.block(0,i+1,dim/2,1) = z_out.block(0,i,dim/2,1) + z_out.block(dim/2,i+1,dim/2,1) * sim_dt;

    }
    delete J;
    Animator animator(parameter);

    animator.animate(z_out);
    

}

vector<double> basic_control(Eigen::VectorXd z0, const Eigen::Ref<const Eigen::MatrixXd>& z){
    double k_th = 20;
    double D_th = 0.8;
    Eigen::VectorXd q_des = z0.block(3,0,3,1);
    Eigen::VectorXd q_cur =  z.block(3,0,3,1);
    Eigen::VectorXd qdot  = z.block(9,0,3,1);
    Eigen::VectorXd tau_ = k_th * (q_des - q_cur) + D_th *(- qdot);
    vector<double> tau(tau_.data(), tau_.data() + tau_.rows() * tau_.cols());
    return tau;
}

casadi::SX centroidal_dynamics(SX state, SX Fx, SX Fy, SX Tau, double M,  double Inertia, double gravity, double ground_height){
    SX x = state(0); SX y = state(1); SX th = state(2); 
    SX dx = state(3); SX dy = state(4); SX dth = state(5);
 
    //Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,3);
    casadi::SX A = casadi::SX::zeros(3,3);
    A(0,0) = M; A(1,1) = M; A(2,2) = Inertia;
    casadi::SX A_inv =  casadi::SX::inv(A);
    casadi::SX temp1 = vertcat(x,y-ground_height, 0);
    casadi::SX temp2 = vertcat(Fx, Fy, 0);

    casadi::SX F = vertcat(Fx, Fy - M*gravity, 0) + casadi::SX::cross(temp1, temp2) + vertcat(0,0,Tau);
    casadi::SX qddot =  casadi::SX::mtimes(A_inv, F);
    casadi::SX ds = vertcat(dx, dy, dth); ds = vertcat(ds, qddot);

    return ds;

}

