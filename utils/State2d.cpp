#include "State2d.hpp"

State2d::State2d(Vector2d x, Vector2d xd, double theta, double w, double reward, double curr_contact_loc):
x(x),
xd(xd),
theta(theta),
w(w),
reward(reward),
curr_contact_loc(curr_contact_loc)
{

}
State2d::State2d(){}

void State2d::operator = (const State2d &s){
    x = s.x;
    xd = s.xd;
    theta = s.theta;
    w = s.w;
    reward = s.reward;
    curr_contact_loc = s.curr_contact_loc;
}