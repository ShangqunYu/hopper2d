#include "eigenHelper.hpp"

Eigen::MatrixXd dmToEigen(vector<DM> &m){
    std::pair<int, int> sizes = m[0].size();
    int row = sizes.first;
    int col = sizes.second;
    std::vector<double> values = m[0].get_elements();
    Eigen::MatrixXd e(row,col);
    for (int i = 0; i < col; i++)
    {
        for (int j = 0; j < row; j++)
        {
            e(j,i)=values[i*row+j];
        }

    }
    return e;
}

casadi::DM EigenTodm(Eigen::MatrixXd matrix){
    size_t rows = matrix.rows();
    size_t cols = matrix.cols();

    casadi::DM casadi_matrix = casadi::DM::zeros(rows,cols);

    std::memcpy(casadi_matrix.ptr(), matrix.data(), sizeof(double)*rows*cols);

    return casadi_matrix;
}

Eigen::MatrixXd arrayToEigen(double m[], int row , int col){

    Eigen::MatrixXd e(row,col);
    for (int i = 0; i < col; i++)
    {
        for (int j = 0; j < row; j++)
        {
            e(j,i)=m[i*row+j];
        }

    }
    return e;
}

void print_hello(){
    cout<<"hello"<<endl;
}