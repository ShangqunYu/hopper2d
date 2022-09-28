#include "Animator.hpp"

Animator::Animator(vector<double> p):
    _lcm_cart("udpm://239.255.76.67:7667?ttl=255"),
    _dim(12),
    _ini_state(12),
    _cur_state(12),
    l0(p[11]),
    l1(p[12]),
    l2(p[13]),
    l21(p[14]),
    lbody(p[16])

{
    _ini_state.setZero();
    _cur_state = _ini_state;
    cout<<"animator loaded" << "l0"<<l0 << "l1"<<l1<< "l2"<<l2<<endl;
    cout<<lbody<<endl;
}


void Animator::animate(Eigen::MatrixXd z)
{
 
    for (int i = 0; i < z.cols(); i++)
    {
        //get the current state
        _cur_state = z.col(i);
        send_message();
        usleep(10000);
        
    }
 
}


void Animator::send_message(){
    double x = _cur_state[0];
    double y = _cur_state[1];
    double theta = _cur_state[2];
    double O_x = x + lbody/2 * sin(theta);
    double O_y = y - lbody/2 * cos(theta);
    double q0 = _cur_state[3];
    double l0_angle =  theta + q0;
    double x_l0 = O_x + l0/2 * sin(l0_angle);
    double y_l0 = O_y -l0/2 * cos(l0_angle);
    double q1 = _cur_state[4];
    double l1_angle = theta + q0 + q1;
    double x_l1 = O_x + l0 * sin(l0_angle) + l1/2 * sin(l1_angle);
    double y_l1 = O_y - l0 * cos(l0_angle) - l1/2 * cos(l1_angle);
    double q2 = _cur_state[5];
    double l2_angle = theta + q0+q1+q2+M_PI/2;
    double x_l1_end = O_x + l0 * sin(l0_angle) + l1 * sin(l1_angle);
    double y_l1_end = O_y - l0 * cos(l0_angle) - l1 * cos(l1_angle);
    double l2_c_to_l1_end = (l2+l21)/2 - l21;
    double x_l2 = x_l1_end + l2_c_to_l1_end * sin(l2_angle);
    double y_l2 = y_l1_end - l2_c_to_l1_end * cos(l2_angle);
    cout<<"x_l1_end: "<<x_l1_end<<" x_l2: "<< x_l2<<endl;
    _kin_msg.torso_pos[0] = x;
    _kin_msg.torso_pos[1] = y;
    _kin_msg.torso_pos[2] = theta;
    
    _kin_msg.link0_pos[0] = x_l0;
    _kin_msg.link0_pos[1] = y_l0;
    _kin_msg.link0_pos[2]= l0_angle;
    _kin_msg.link1_pos[0] = x_l1;
    _kin_msg.link1_pos[1] = y_l1;
    _kin_msg.link1_pos[2]= l1_angle;
    _kin_msg.link2_pos[0] = x_l2;
    _kin_msg.link2_pos[1] = y_l2;
    _kin_msg.link2_pos[2]= l2_angle;

    _lcm_cart.publish("hopper2d", &_kin_msg);
}
