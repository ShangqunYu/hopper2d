#include "dynamics.hpp"
using namespace casadi;


Eigen::VectorXd dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus){
    
    cout<<"torques: "<<taus[0]<<", "<<taus[1]<<", "<<taus[2]<<endl;
    vector<double> z_vec(z.data(), z.data() + z.rows() * z.cols());    
    double* z_vecp = &z_vec[0];
    double* paramp = &p[0];
    double* tausp = &taus[0];

    //getting A matrix
    Eigen::MatrixXd A_matrix = get_A_matrix(z_vecp, paramp);

    //getting B matrix
    Eigen::MatrixXd b_matrix = get_b_matrix(z_vecp, tausp, paramp);
 
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

// runge kutta method
Eigen::VectorXd rk4(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus, double dt){
    Eigen::VectorXd k1 = dynamics(z, p, taus);
    Eigen::VectorXd k2 = dynamics(z + dt/2*k1, p, taus);
    Eigen::VectorXd k3 = dynamics(z + dt/2*k2, p, taus);
    Eigen::VectorXd k4 = dynamics(z + dt*k3, p, taus);
    Eigen::VectorXd dz = dt/6*(k1 + 2*k2 + 2*k3 + k4);

    //return next state
    return z + dz;    
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

            double dF_y =  lambda_y * (-rest_coeff * vB(1) - (J_matrix_y * qdot)(0));  //using 0 to convert from 1x1 matrix to double
            //having a cap, the overall force can't be negative
            if (Fy(contact_pts[contact_idx]) + dF_y < 0 ){
                dF_y = -Fy(contact_pts[contact_idx]);
            }
            Fy(contact_pts[contact_idx]) = Fy(contact_pts[contact_idx]) + dF_y;
            qdot = qdot + A_matrix_inv*J_matrix_y.transpose() * dF_y;
            //Horizontal
            Eigen::MatrixXd J_matrix_x = J_matrix(0, Eigen::placeholders::all);
            double lambda_x = 1/(J_matrix_x * A_matrix_inv * J_matrix_x.transpose()).coeff(0,0);
            
            double dF_x =  lambda_x * (0 - (J_matrix_x * qdot)(0));  //using 0 to convert from 1x1 matrix to double
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

Eigen::VectorXd discrete_contact_dynamics_new(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, double rest_coeff, double fric_coeff, double ground_height){
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

    int num_iteration = 20;
    
    Eigen::MatrixXd A_matrix = get_A_matrix(z_vecp, paramp);
    //taking the inverse of A matrix
    Eigen::MatrixXd A_matrix_inv = A_matrix.inverse();
    vector<int> contact_pts;
    vector<Eigen::VectorXd> contact_pts_vel;
    vector<double> contact_pts_post_vel_y;
    vector<Eigen::MatrixXd> contact_pts_jacobian;
    vector<Eigen::MatrixXd> contact_pts_jacobian_y;
    vector<Eigen::MatrixXd> contact_pts_jacobian_x;
    vector<double> contact_pts_lambda_y;
    vector<double> contact_pts_lambda_x;
    for (int k = 0; k < num_contact_pts; k++){
        if(pose_contact_matrix(1,k)-ground_height<0){
            contact_pts.push_back(k);
            Eigen::VectorXd pt_vel = vel_contact_matrix(Eigen::placeholders::all, k);
            contact_pts_vel.push_back(pt_vel);

            double post_vel_y = (pt_vel(1) < 0) ? -rest_coeff * pt_vel(1): pt_vel(1);
            contact_pts_post_vel_y.push_back(post_vel_y);

            double* J; J = new double [18];
            jacobians(z_vecp, paramp, k, J);
            Eigen::MatrixXd J_matrix = arrayToEigen(J, 3, 6);
            contact_pts_jacobian.push_back(J_matrix);

            Eigen::MatrixXd J_matrix_y = J_matrix(1, Eigen::placeholders::all);
            contact_pts_jacobian_y.push_back(J_matrix_y);

            Eigen::MatrixXd J_matrix_x = J_matrix(0, Eigen::placeholders::all);
            contact_pts_jacobian_x.push_back(J_matrix_x);

            double lambda_y = 1/(J_matrix_y * A_matrix_inv * J_matrix_y.transpose()).coeff(0,0);
            contact_pts_lambda_y.push_back(lambda_y);

            double lambda_x = 1/(J_matrix_x * A_matrix_inv * J_matrix_x.transpose()).coeff(0,0);
            contact_pts_lambda_x.push_back(lambda_x);
            delete J;
        }
    }
    // init as ones
    Eigen::VectorXd dF_ys = Eigen::VectorXd::Ones(contact_pts.size()) * 1000;
    Eigen::VectorXd dF_xs = Eigen::VectorXd::Ones(contact_pts.size()) * 1000;

    double tol = 0.0001;

    for (int i = 0; i< num_iteration; i++){
        //if the abs sum of dF_ys is smaller than 0.001, break
        if (dF_ys.cwiseAbs().sum() < tol && dF_xs.cwiseAbs().sum() < tol){
            break;
        }

        //cout << endl;
        for (int contact_idx = 0; contact_idx < contact_pts.size(); contact_idx++){
            
            Eigen::VectorXd vB = contact_pts_vel[contact_idx];
 
            Eigen::MatrixXd J_matrix = contact_pts_jacobian[contact_idx];

            //Vertical
            Eigen::MatrixXd J_matrix_y = contact_pts_jacobian_y[contact_idx];
            double lambda_y = contact_pts_lambda_y[contact_idx];
            
            double vy_post = contact_pts_post_vel_y[contact_idx];
            dF_ys[contact_idx] =  lambda_y * (vy_post - (J_matrix_y * qdot)(0));  //using 0 to convert from 1x1 matrix to double
            //having a cap, the overall force can't be negative
            double next_Fy = max(0.0, Fy(contact_pts[contact_idx])+ dF_ys[contact_idx]);
            dF_ys(contact_idx) = next_Fy - Fy(contact_pts[contact_idx]);
            Fy(contact_pts[contact_idx]) = Fy(contact_pts[contact_idx]) + dF_ys[contact_idx];
            qdot = qdot + A_matrix_inv*J_matrix_y.transpose() * dF_ys[contact_idx];

            //Horizontal
            Eigen::MatrixXd J_matrix_x = contact_pts_jacobian_x[contact_idx];
            double lambda_x = contact_pts_lambda_x[contact_idx];
            double vx_post = 0;
            dF_xs(contact_idx) =  lambda_x * (vx_post - (J_matrix_x * qdot)(0));  //using 0 to convert from 1x1 matrix to double
            //check if the overall tengential force is outside of the friction cone
            double next_Fx = max (-fric_coeff * Fy(contact_pts[contact_idx]), min(fric_coeff * Fy(contact_pts[contact_idx]), Fx(contact_pts[contact_idx])+ dF_xs(contact_idx)));
            dF_xs(contact_idx) = next_Fx - Fx(contact_pts[contact_idx]);
            Fx(contact_pts[contact_idx]) = Fx(contact_pts[contact_idx]) + dF_xs(contact_idx);
            qdot = qdot + A_matrix_inv*J_matrix_x.transpose() * dF_xs(contact_idx);
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


