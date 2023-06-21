#include <Eigen/Core>

using namespace Eigen;
class Parameters {
    public:
        Parameters();
        ~Parameters(){}

        //robot property
        double mbody, m0, m1, m2, M;  

        double Ibody, I0, I1, I2; // inertia

        double l0, l1, l2, l21, lbody; // length of link

        double c0, c1, c2; // length to center of mass

        double gravity; 

        double ground_height;
        
        double rest_coeff; 
        double fric_coeff;
        double dt;
        int dim;
        // declare a 12 dim eigen vector
        VectorXd init_state;

        double kp, kd;

        Vector3d max_torque;

        double terminal_height;
        double terminal_width;
        double terminal_angle;
        double terminal_thetas;
        double healthy_state_range;

        int max_steps;
        int time_skipping;
};