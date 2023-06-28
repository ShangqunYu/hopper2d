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
        void step(VectorXd actions) {
            // env.step(actions);
        }

        VectorXd get_obs(){
            VectorXd obs = VectorXd::Zero(
                  env.s.x.size() - 1      // y position of the body
                + env.s.xd.size()         // xy vel of the body
                + 1                       // theta
                + 1                      // theta dot
                + 1                     // current contact location
                + 1                     // reward
                + 1                     // done
            );
            obs << env.s.x(1), env.s.xd, env.s.theta, env.s.w, env.s.curr_contact_loc, env.s.reward, env.log.done;
        }
        
        void render() {
            env.render();
        }

        void reset(){
            env.reset();
        }

        // double calc_reward(){
        //     return env.calc_jump_reward();
        // }

        // bool is_done(){
        //     return env.is_done();
        // }

        hopper2dOptiEnv env;
};


PYBIND11_MODULE(hopper2dOptiWrapper, m) {
    py::class_<Hopper2dOptiWrapper>(m, "Hopper2dOptiWrapper")
        .def(py::init<>())
        .def("step", &Hopper2dOptiWrapper::step)
        .def("render", &Hopper2dOptiWrapper::render)
        .def("reset", &Hopper2dOptiWrapper::reset);
        // .def("calc_reward", &Hopper2dOptiWrapper::calc_reward)
        // .def("is_done", &Hopper2dOptiWrapper::is_done);

}