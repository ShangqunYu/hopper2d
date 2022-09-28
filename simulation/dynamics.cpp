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