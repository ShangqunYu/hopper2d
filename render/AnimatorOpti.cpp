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
        send_message(x, y, theta, log.cd.cl(0, i-1), log.cd.cs[i-1]);
        usleep(100000);
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


void AnimatorOpti::animate_loco(loco_logdata log)
{

    for (int i = 1; i < log.x.size2(); i++)
    {
        double x = log.x(0,i).scalar();
        double y = log.x(1,i).scalar();
        double theta = log.theta(0,i).scalar();
        send_message_loco(x, y, theta, log.cd.rcl(0, i-1), log.cd.rcs[i-1], log.cd.lcl(0, i-1), log.cd.lcs[i-1]);
        usleep(100000);
    }

}

void AnimatorOpti::send_message_loco(double x, double y, double theta, double r_contact_loc, double r_in_contact, double l_contact_loc, double l_in_contact){
    loco2dOpti_lcmt msg;
    msg.body_pos[0] = x;
    msg.body_pos[1] = y;
    msg.theta = theta;
    msg.r_contact_pos = r_contact_loc;
    msg.r_under_contact = r_in_contact;
    msg.l_contact_pos = l_contact_loc;
    msg.l_under_contact = l_in_contact;
    _lcm_cart.publish("loco2dOpti", &msg);
}


void AnimatorOpti::send_terrain(VectorXd terrain){
    terrain_lcmt msg;
    for (int i = 0; i < terrain.size(); i++){
        msg.floor[i] = terrain[i];
    }
    _lcm_cart.publish("terrain", &msg);
}