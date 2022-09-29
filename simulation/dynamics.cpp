#include "dynamics.hpp"
using namespace casadi;


Eigen::VectorXd dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus){
    
    // Function f_A = external("A", "../lib/A.so"); 
    vector<double> z_vec(z.data(), z.data() + z.rows() * z.cols());
    // vector<DM> arg_A = {DM(z_vec), DM(p)};
    // vector<DM> A = f_A(arg_A); 
    // Eigen::MatrixXd A_matrix  = dmToEigen(A); 

 
    
    double* z_vecp = &z_vec[0];
    double* paramp = &p[0];
    double* tausp = &taus[0];

    //getting A matrix
    double* A_container;
    A_container = new double [36];
    A_hopper(z_vecp, paramp, A_container);
    Eigen::MatrixXd A_matrix = arrayToEigen(A_container, 6, 6);

    //getting B matrix
    double* b_container;
    b_container = new double [6];
    b_hopper(z_vecp, tausp, paramp, b_container);
    
    //cout<<"matrix size:"<<b_matrix.rows()<<" "<<b_matrix.cols()<<endl;
    Eigen::MatrixXd b_matrix = arrayToEigen(b_container, 6, 1);
    //cout<< b_matrix_temp << endl;
    //cout<< "compare:"<<endl;
    //cout<<b_matrix<<endl;
    

    //taking the inverse of A matrix
    Eigen::MatrixXd A_matrix_inv = A_matrix.inverse();

    //getting the q double dot
    Eigen::MatrixXd qdd = A_matrix_inv * b_matrix;

    //create dz
    int dim = z.size();
    Eigen::VectorXd dz = Eigen::VectorXd::Zero(dim);

    dz.segment(0,dim/2) = z.block(dim/2,0,dim/2,1);
    dz.segment(dim/2,dim/2) = qdd;


    delete b_container;
    return dz;
}


Eigen::VectorXd discrete_contact_dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, double rest_coeff, double fric_coeff, double ground_height){
    int dim = z.size();
    // unpack z_vec
    vector<double> z_vec(z.data(), z.data() + z.rows() * z.cols());
    // get qdot
    vector<double> qdot = std::vector<double>(z_vec.begin() + dim/2, z_vec.end());
    // get contact point position and velocity
    double* z_vecp = &z_vec[0];
    double* paramp = &p[0];
    double* contact_point_pose; double* contact_point_vel;
    int num_contact_pts = 9;
    contact_point_pose = new double [num_contact_pts*3]; contact_point_vel = new double [num_contact_pts*3];
    position_contact_points(z_vecp, paramp, contact_point_pose);
    velocity_contact_points(z_vecp, paramp, contact_point_vel); 
    Eigen::MatrixXd pose_contact_matrix = arrayToEigen(contact_point_pose, num_contact_pts, 3);
    Eigen::MatrixXd vel_contact_matrix = arrayToEigen(contact_point_vel, num_contact_pts, 3);

    Eigen::VectorXd Fz = Eigen::VectorXd::Zero(num_contact_pts);
    Eigen::VectorXd Fx = Eigen::VectorXd::Zero(num_contact_pts);
    Eigen::VectorXd dz_ = Eigen::VectorXd::Zero(dim);

    int num_iteration = 10;
    
    
    for (int i = 0; i< num_iteration; i++){
        vector<int> contact_pts;
        for (int k = 0; k < num_contact_pts; k++){
            if(pose_contact_matrix(k,1)-ground_height<0 &&   vel_contact_matrix(k,1) < 0){
                contact_pts.push_back(k);
            }
        }

        if (contact_pts.size()<1){
            cout<<"nothing!"<<endl;
            break;
        }

        for (int contact_idx = 0; contact_idx < contact_pts.size(); contact_idx++){
            Eigen::VectorXd vB = vel_contact_matrix(contact_pts[contact_idx] ,Eigen::placeholders::all);
            double* J; J = new double [18];
            jacobians(z_vecp, paramp, contact_pts[contact_idx], J);
            Eigen::MatrixXd J_matrix = arrayToEigen(J, 6, 3);
            //you need to implement matlab reshape function!
            cout<< contact_pts[contact_idx]<< endl<<J_matrix<<endl;










            delete J;
        }

    }

    delete contact_point_pose; delete contact_point_vel;
    return dz_;
}