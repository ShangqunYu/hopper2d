#include "AnimatorOpti.hpp"

AnimatorOpti::AnimatorOpti():
    _lcm_cart("udpm://239.255.76.67:7667?ttl=255")
{

}


void AnimatorOpti::animate(logdata log)
{

    for (int i = 1; i < log.x.size2(); i++)
    {
        double x = log.x(0,i).scalar();
        double y = log.x(1,i).scalar();
        double theta = log.theta(0,i).scalar();
        send_message(x, y, theta, theta, log.cd.cs[i-1]);
        usleep(10000);
    }

}




void AnimatorOpti::send_message(double x, double y, double theta, double contact_loc, double in_contact){
    hopper2dOpti_lcmt msg;
    msg.body_pos[0] = x;
    msg.body_pos[1] = y;
    msg.theta = theta;
    msg.contact_pos = contact_loc;
    msg.under_contact = in_contact;
    _lcm_cart.publish("hopper2dOpti", &msg);
}
