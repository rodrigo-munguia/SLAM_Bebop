#include "speed_update.h"
#include "math.h"
#include "../Transforms/quat2R.h"

/*
% Initial States ***************************************************
% x meaning
% index  0  1  2  3  4   5   6   7  8  9  10  11  12  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
*/


void speed_Update(arma::vec& x, arma::mat& P,SPD &speed,parameters &par)
{

    std::vector<double> quat;
    quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
    double Rn2c_a[9];
    quat2R(&quat[0],Rn2c_a);
    arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo

    arma::vec::fixed<3> z;  // measurement
    // account for Bebop to camera coordinate transformation
    z(0) = speed.speedY;  
    z(1) = -speed.speedX;
    z(2) = -speed.speedZ;

    arma::vec::fixed<3> s_c = x.subvec(10,12); // (velocity) speed vector in vnavigation frame
    
    arma::vec::fixed<3> h = Rn2c*s_c;  // measurement prediction

    

    //--------------- Measurement Jacobian
    int x_len = x.size();
    arma::mat H;
    // Form jacobian
    H.resize(3,x_len);
    arma::mat::fixed<3,4> dPc_dq; 
    double q1 = x(0);
    double q2 = x(1);
    double q3 = x(2);
    double q4 = x(3); 
    double Pn1 = x(10);  // camera velocity in x (nav frame)
    double Pn2 = x(11);  // camera velocity in y (nav frame)
    double Pn3 = x(12);  // camera velocity in z (nav frame)

   dPc_dq = {{ 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn2*q4 - 2*Pn3*q3 - 2*Pn1*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4},
                  { 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn3*q2 - 2*Pn2*q1 - 2*Pn1*q4},
                  { 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn1*q3 - 2*Pn2*q2 - 2*Pn3*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4}};

    H(arma::span(0,2),arma::span(0,3)) = dPc_dq;
    H(arma::span(0,2),arma::span(10,12)) = Rn2c;
   //-------------------------------------------- 
    
   double vs = par.ekf.sigma_sp*par.ekf.sigma_sp;  // variance
   arma::mat::fixed<3,3> R;
   R(0,0) = vs;
   R(1,1) = vs;
   R(2,2) = vs*10;


    arma::mat H_P = H*P;

    arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
    arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

    P = P - K*H_P;  // System Covariance matrix update 
 
    x = x + K*(z-h);  // System vector update

    //----------------------
    
    /*
    cout << "Measured: speedX: " << z(0) << " speedY: " << z(1) << " speedZ:" << z(2) << endl;
    arma::vec::fixed<3> s_u = x.subvec(10,12); // (velocity) speed vector in vnavigation frame
    arma::vec::fixed<3> v = Rn2c*s_u;  // measurement prediction
    cout << "   State: speedX: " << v(0) << " speedY: " << v(1) << " speedZ:" << v(2) << endl;
    */
}