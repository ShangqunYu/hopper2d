#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "../simulation/hopper2dOptiEnv.hpp"
#include <iostream>
using namespace std;

namespace py = pybind11;


class Hopper2dOptiWrapper {
    public:
        Hopper2dOptiWrapper() {

            cout << "Hopper2dOptiWrapper constructor" << endl;
        }
        ~Hopper2dOptiWrapper() {
            cout << "Hopper2dOptiWrapper destructor" << endl;
        }
        VectorXd step(VectorXd actions) {
            env.step(actions(0), actions(1), actions(2));
            return get_obs();
        }

        VectorXd get_obs(){
            VectorXd obs = VectorXd::Zero(
                  env.s.x.size()       //x position relative to contact and y position of the body
                + env.s.xd.size()         // xy vel of the body
                + 1                       // theta
                + 1                      // theta dot
            );
            obs << env.s.x(0) - env.s.curr_contact_loc, env.s.x(1), env.s.xd, env.s.theta, env.s.w;
            return obs;
        }

        double calc_reward(){
            return env.s.reward;
        }

        bool is_done(){
            return env.log.done;
        }
        
        void render() {
            env.render();
        }

        VectorXd reset(){
            env.reset();
            return get_obs();
        }

        hopper2dOptiEnv env;
};


PYBIND11_MODULE(hopper2dOptiWrapper, m) {
    py::class_<Hopper2dOptiWrapper>(m, "Hopper2dOptiWrapper")
        .def(py::init<>())
        .def("step", &Hopper2dOptiWrapper::step)
        .def("render", &Hopper2dOptiWrapper::render)
        .def("reset", &Hopper2dOptiWrapper::reset)
        .def("calc_reward", &Hopper2dOptiWrapper::calc_reward)
        .def("is_done", &Hopper2dOptiWrapper::is_done);
}