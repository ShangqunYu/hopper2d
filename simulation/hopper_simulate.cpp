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
    int n_state = 6; int n_control = 3;
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
    casadi::Function central_dynamics = central_model_dynamics(mbody,  m0,  m1,  m2,  Ibody,  gravity,  ground_height);
    cout<<central_dynamics<<endl;
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
        cout<< ds <<endl;
        
    }







    for(int i = 0; i < num_sim_step-1; i++){
        taus =  basic_control(z0, z_out.block(0,i,dim,1));
        Eigen::VectorXd dz = dynamics(z_out.block(0,i,dim,1), parameter, taus);
        
        z_out.block(0,i+1,dim,1) = z_out.block(0,i,dim,1) + dz * sim_dt;
        z_out.block(dim/2,i+1,dim/2,1)= discrete_contact_dynamics(z_out.block(0,i+1,dim,1), parameter, rest_coeff, fric_coeff, ground_height);
        z_out.block(0,i+1,dim/2,1) = z_out.block(0,i,dim/2,1) + z_out.block(dim/2,i+1,dim/2,1) * sim_dt;
        //cout<< "i: "<<i<<endl;
        //break;
    }
    
    //cout<< z_out.block(0,num_sim_step-20,dim,10);
    
    Animator animator(parameter);

    animator.animate(z_out);
    

}

vector<double> basic_control(Eigen::VectorXd z0, const Eigen::Ref<const Eigen::MatrixXd>& z){
    double k_th = 50;
    double D_th = 0.05;
    Eigen::VectorXd q_des = z0.block(3,0,3,1);
    Eigen::VectorXd q_cur =  z.block(3,0,3,1);
    Eigen::VectorXd qdot  = z.block(0,0,3,1);
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


Function central_model_dynamics(double mbody, double m0, double m1, double m2, double Ibody, double gravity, double ground_height){
    double M = mbody + m0 + m1 + m2; double Inertia = Ibody;
    SX x = SX::sym("x"); SX y = SX::sym("y"); SX th = SX::sym("th"); 
    SX dx = SX::sym("dx"); SX dy = SX::sym("dy"); SX dth = SX::sym("dth");
    SX Fx = SX::sym("Fx"); SX Fy = SX::sym("Fy"); SX Tau = SX::sym("tau");
    SX state = vertcat(x,y,th); state =vertcat(state, dx,dy); state = vertcat(state, dth);
    cout<<"I am here!:" << state.size()<<endl;

    //Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,3);
    casadi::SX A = casadi::SX::zeros(3,3);
    A(0,0) = M; A(1,1) = M; A(2,2) = Inertia;
    casadi::SX A_inv =  casadi::SX::inv(A);
    casadi::SX temp1 = vertcat(x,y-ground_height, 0);
    casadi::SX temp2 = vertcat(Fx, Fy, 0);

    casadi::SX F = vertcat(Fx, Fy - M*gravity, 0) + casadi::SX::cross(temp1, temp2) + vertcat(0,0,Tau);
    casadi::SX qddot =  casadi::SX::mtimes(A_inv, F);
    casadi::SX ds = vertcat(dx, dy, dth); ds = vertcat(ds, qddot);


    return Function("centroidal_dynamics", {x, y, th, dx, dy, dth, Fx, Fy, Tau}, {ds});

}