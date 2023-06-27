#include "eigenHelper.hpp"

Eigen::MatrixXd dmToEigen(casadi::DM &m)
{
    int rows = m.size1();
    int cols = m.size2();
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            out(i, j) = m(i, j).scalar();
        }
    }
    return out;

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

casadi::DM EigenVectorTodm(Eigen::VectorXd vec)
{
    size_t rows = vec.rows();
    casadi::DM casadi_vec = casadi::DM::zeros(rows, 1);
    for (int i = 0; i < rows; i++)
    {
        casadi_vec(i, 0) = vec(i);
    }
    return casadi_vec;
}

casadi::DM EigenMatrixTodm(Eigen::MatrixXd matrix)
{
    size_t rows = matrix.rows();
    size_t cols = matrix.cols();
    casadi::DM casadi_matrix = casadi::DM::zeros(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            casadi_matrix(i, j) = matrix(i, j);
        }
    }
    return casadi_matrix;
}