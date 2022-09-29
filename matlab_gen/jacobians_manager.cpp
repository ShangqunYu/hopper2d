#include "jacobians_manager.hpp"


void jacobians(const double in1[12], const double in2[17], int i, double J_container[18]){

    switch(i){
        case 0:
            jacobian_rbody(in1, in2, J_container);
            break;
        case 1:
            jacobian_a(in1, in2, J_container);
            break;
        case 2:
            jacobian_e(in1, in2, J_container);
            break;
        case 3:
            jacobian_b(in1, in2, J_container);
            break;
        case 4:
            jacobian_c(in1, in2, J_container);
            break;
        case 5:
            jacobian_k(in1, in2, J_container);
            break;
        case 6:
            jacobian_j(in1, in2, J_container);
            break;
        case 7:
            jacobian_i(in1, in2, J_container);
            break;
        case 8:
            jacobian_h(in1, in2, J_container);
            break;
    }
}