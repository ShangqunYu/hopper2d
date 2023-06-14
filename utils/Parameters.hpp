#include <Eigen/Core>

class Parameters {
    public:
        Parameters();
        ~Parameters(){}

        //robot property
        double mbody = 4; double m0 = 0.3; double m1 = 0.5; double m2 = 0.1;  
        double M = mbody + m0 + m1 + m2; // Total Mass

        double Ibody = 11 * pow(10,-3); double I0 = 5.1 * pow(10,-6); double I1 = 5.1 * pow(10,-6); double I2 = 1.5 * pow(10,-6); // inertia

        double l0 = 0.2; double l1 = 0.22; double l2 = 0.1;  double l21 = 0.06; double lbody = 0.3; // length of link

        double c0 = 0.1; double c1 = 0.1; double c2 = 0.07; // length to center of mass

        double gravity = 9.81; 

        double ground_height = 0;
        
        double rest_coeff = 0.2; 
        double fric_coeff = 0.7;
};