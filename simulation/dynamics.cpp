#include "dynamics.hpp"
using namespace casadi;


Eigen::VectorXd dynamics(const Eigen::Ref<const Eigen::MatrixXd>& z, vector<double> &p, vector<double> &taus){
    //getting A matrix
    Function f_A = external("A", "../lib/A.so"); 
    vector<double> z_vec(z.data(), z.data() + z.rows() * z.cols());
    vector<DM> arg_A = {DM(z_vec), DM(p)};
    vector<DM> A = f_A(arg_A); 
    Eigen::MatrixXd A_matrix  = dmToEigen(A); 

 
    //getting B matrix
    Function f_b = external("b", "../lib/b.so"); 
    vector<DM> arg_b = {DM(z_vec), DM(taus), DM(p)};
    vector<DM> b = f_b(arg_b);
    Eigen::MatrixXd b_matrix  = dmToEigen(b); 
    double* z_vecp = &z_vec[0];
    double* tausp = &taus[0];
    double* pp = &p[0];
    double* container;
    container = new double [6];
    //std::unique_ptr<int[]>  container(new double[6]);
    b_hopper(z_vecp, tausp, pp, container);
    
    //cout<<"matrix size:"<<b_matrix.rows()<<" "<<b_matrix.cols()<<endl;
    Eigen::MatrixXd b_matrix_temp = arrayToEigen(container, 6, 1);
    //cout<< b_matrix_temp << endl;
    //cout<< "compare:"<<endl;
    //cout<<b_matrix<<endl;
    

    //taking the inverse of A matrix
    Eigen::MatrixXd A_matrix_inv = A_matrix.inverse();

    //getting the q double dot
    Eigen::MatrixXd qdd = A_matrix_inv * b_matrix_temp;

    //create dz
    int dim = z.size();
    Eigen::VectorXd dz = Eigen::VectorXd::Zero(dim);

    dz.segment(0,dim/2) = z.block(dim/2,0,dim/2,1);
    dz.segment(dim/2,dim/2) = qdd;


    delete container;
    return dz;
}