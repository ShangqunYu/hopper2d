#include "eigenHelper.hpp"

Eigen::MatrixXd dmToEigen(vector<DM> &m){
    std::pair<int, int> sizes = m[0].size();
    int row = sizes.first;
    int col = sizes.second;
    std::vector<double> values = m[0].get_elements();
    Eigen::MatrixXd e(row,col);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            e(i,j)=values[i*col+j];
        }

    }
    return e;
}

Eigen::MatrixXd arrayToEigen(double m[], int row , int col){

    Eigen::MatrixXd e(row,col);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            e(i,j)=m[i*col+j];
        }

    }
    return e;
}

void print_hello(){
    cout<<"hello"<<endl;
}