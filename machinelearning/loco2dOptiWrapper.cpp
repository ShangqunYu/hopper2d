#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "../simulation/loco2dOptiEnv.hpp"
#include <iostream>
using namespace std;

namespace py = pybind11;


class Loco2dOptiWrapper {
    public:
        Loco2dOptiWrapper() {

            cout << "loco2dOptiWrapper constructor" << endl;
        }
        ~Loco2dOptiWrapper() {
            cout << "Loco2dOptiWrapper destructor" << endl;
        }
        VectorXd step(VectorXd actions) {
            env.step(actions(0), actions(1), actions(2), actions(3), actions(4), actions(5));
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

        loco2dOptiEnv env;
};


PYBIND11_MODULE(loco2dOptiWrapper, m) {
    py::class_<Loco2dOptiWrapper>(m, "Loco2dOptiWrapper")
        .def(py::init<>())
        .def("step", &Loco2dOptiWrapper::step)
        .def("render", &Loco2dOptiWrapper::render)
        .def("reset", &Loco2dOptiWrapper::reset)
        .def("calc_reward", &Loco2dOptiWrapper::calc_reward)
        .def("is_done", &Loco2dOptiWrapper::is_done);
}