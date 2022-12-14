#include "dynamics.hpp"
using namespace casadi;


Eigen::VectorXd dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus){
    

    vector<double> z_vec(z.data(), z.data() + z.rows() * z.cols());    
    double* z_vecp = &z_vec[0];
    double* paramp = &p[0];
    double* tausp = &taus[0];

    //getting A matrix
    Eigen::MatrixXd A_matrix = get_A_matrix(z_vecp, paramp);

    //getting B matrix
    Eigen::MatrixXd b_matrix = get_b_matrix(z_vecp, tausp, paramp);
 
    //cout<<A_matrix<<endl;
    

    //taking the inverse of A matrix
    Eigen::MatrixXd A_matrix_inv = A_matrix.inverse();

    //getting the q double dot
    Eigen::MatrixXd qdd = A_matrix_inv * b_matrix;

    //create dz
    int dim = z.size();
    Eigen::VectorXd dz = Eigen::VectorXd::Zero(dim);

    dz.segment(0,dim/2) = z.block(dim/2,0,dim/2,1);
    dz.segment(dim/2,dim/2) = qdd;

    return dz;
}


Eigen::VectorXd discrete_contact_dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, double rest_coeff, double fric_coeff, double ground_height){
    int dim = z.size();
    // unpack z_vec
    vector<double> z_vec(z.data(), z.data() + z.rows() * z.cols());
    // get qdot
    Eigen::VectorXd qdot = z.block<6,1>(6,0);

    // get contact point position and velocity
    double* z_vecp = &z_vec[0];
    Eigen::VectorXd temp = z;

    double* paramp = &p[0];
    double* contact_point_pose; double* contact_point_vel;
    int num_contact_pts = 9;
    contact_point_pose = new double [num_contact_pts*3]; contact_point_vel = new double [num_contact_pts*3];
    position_contact_points(z_vecp, paramp, contact_point_pose);
    velocity_contact_points(z_vecp, paramp, contact_point_vel); 
 
    Eigen::MatrixXd pose_contact_matrix = arrayToEigen(contact_point_pose, 3, num_contact_pts);
    Eigen::MatrixXd vel_contact_matrix =  arrayToEigen(contact_point_vel, 3, num_contact_pts);

    Eigen::VectorXd Fy = Eigen::VectorXd::Zero(num_contact_pts);
    Eigen::VectorXd Fx = Eigen::VectorXd::Zero(num_contact_pts);
    Eigen::VectorXd dz_ = Eigen::VectorXd::Zero(dim);

    int num_iteration = 10;
    
    Eigen::MatrixXd A_matrix = get_A_matrix(z_vecp, paramp);
    //taking the inverse of A matrix
    Eigen::MatrixXd A_matrix_inv = A_matrix.inverse();
    for (int i = 0; i< num_iteration; i++){
        vector<int> contact_pts;
        for (int k = 0; k < num_contact_pts; k++){
            if(pose_contact_matrix(1,k)-ground_height<0 &&   vel_contact_matrix(1,k) < 0){
                contact_pts.push_back(k);
            }
        }

        if (contact_pts.size()<1){
            break;
        }

        //cout << endl;
        for (int contact_idx = 0; contact_idx < contact_pts.size(); contact_idx++){
            Eigen::VectorXd vB = vel_contact_matrix(Eigen::placeholders::all, contact_pts[contact_idx]);
            double* J; J = new double [18];
            jacobians(z_vecp, paramp, contact_pts[contact_idx], J);
            Eigen::MatrixXd J_matrix = arrayToEigen(J, 3, 6);
            Eigen::MatrixXd J_matrix_y = J_matrix(1, Eigen::placeholders::all);
            double lambda_y = 1/(J_matrix_y * A_matrix_inv * J_matrix_y.transpose()).coeff(0,0);

            double dF_y =  lambda_y * (-rest_coeff * vB(1) - (J_matrix_y * qdot)(0));
            //having a cap, the overall force can't be negative
            if (Fy(contact_pts[contact_idx]) + dF_y < 0 ){
                dF_y = -Fy(contact_pts[contact_idx]);
            }
            Fy(contact_pts[contact_idx]) = Fy(contact_pts[contact_idx]) + dF_y;
            qdot = qdot + A_matrix_inv*J_matrix_y.transpose() * dF_y;
            //Horizontal
            Eigen::MatrixXd J_matrix_x = J_matrix(0, Eigen::placeholders::all);
            double lambda_x = 1/(J_matrix_x * A_matrix_inv * J_matrix_x.transpose()).coeff(0,0);
            
            double dF_x =  lambda_x * (0 - (J_matrix_x * qdot)(0));
            //check if the overall tengential force is outside of the friction cone
            if ( abs(Fx(contact_pts[contact_idx])+ dF_x) > fric_coeff * Fy(contact_pts[contact_idx])  ){
                //getting the sign of the horizontal force
                int sign = ((Fx(contact_pts[contact_idx])+ dF_x)>0) - ((Fx(contact_pts[contact_idx])+ dF_x)<0);
                dF_x = sign * Fy(contact_pts[contact_idx]) * fric_coeff - Fx(contact_pts[contact_idx]);
            }
            Fx(contact_pts[contact_idx]) = Fx(contact_pts[contact_idx]) + dF_x;
            qdot = qdot + A_matrix_inv*J_matrix_x.transpose() * dF_x;
            Eigen::MatrixXd temp_z = z;
            temp_z.block(dim/2,0,dim/2,1) = qdot;
            vector<double> temp_zvec(temp_z.data(), temp_z.data() + temp_z.rows() * temp_z.cols());
            double* tempz_vecp = &temp_zvec[0];
            velocity_contact_points(tempz_vecp, paramp, contact_point_vel); 
            delete J;
        }

    }

    delete contact_point_pose; delete contact_point_vel;
    return qdot;
}

Eigen::MatrixXd get_A_matrix (const double z_vecp[12], const double paramp[17]){

    double* A_container;
    A_container = new double [36];
    A_hopper(z_vecp, paramp, A_container);
    Eigen::MatrixXd A_matrix = arrayToEigen(A_container, 6, 6);
    delete A_container;
    return A_matrix;
}


Eigen::MatrixXd get_b_matrix (const double z_vecp[12], const double tausp[3], const double paramp[17]){

    double* b_container;
    b_container = new double [6];
    b_hopper(z_vecp, tausp, paramp, b_container);
    Eigen::MatrixXd b_matrix = arrayToEigen(b_container, 6, 1);
    delete b_container;
    return b_matrix;
}

