#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "../simulation/hopper2dEnv.hpp"
#include <iostream>
using namespace std;

namespace py = pybind11;


class Hopper2dWrapper {
    public:
        Hopper2dWrapper() {

            cout << "Hopper2dWrapper constructor" << endl;
        }
        ~Hopper2dWrapper() {
            cout << "Hopper2dWrapper destructor" << endl;
        }
        VectorXd step(VectorXd actions) {
            return env.step(actions);
        }
        
        void render() {
            env.render();
            cout<<"Hopper2dWrapper render"<<endl;
        }


        VectorXd reset(){
            return env.reset();
        }

        Hopper2dEnv env;
};


PYBIND11_MODULE(hopper2dWrapper, m) {
    py::class_<Hopper2dWrapper>(m, "Hopper2dWrapper")
        .def(py::init<>())
        .def("step", &Hopper2dWrapper::step)
        .def("render", &Hopper2dWrapper::render)
        .def("reset", &Hopper2dWrapper::reset);

}