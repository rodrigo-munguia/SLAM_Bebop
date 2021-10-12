#include "attitude_update.h"
#include <cmath>
#include "../Transforms/AngleWrap.h"


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


/*
void Attitude_Update(arma::vec& x, arma::mat& P,ATT &att,parameters &par, double &yaw_at_home)
{
  
  
   
   double measured_yaw = att.yaw;
    
  
        double p_ini = par.init.roll_init;
            arma::vec::fixed<3> z;  // measurement
                // account for Bebop to camera coordinate transformation
                z(0) = (att.pitch + p_ini ); 
                z(1) = -att.roll;
                
                double yaw_z = -att.yaw - yaw_at_home;

                AngleWrap(yaw_z);
                
                z(0) = 0;
                z(1) = 0;

                z(2) = yaw_z;
                
                //z(2) = 120*(3.1416/180);
                
                //z(2) = 0;

            Quaternion q;
            q.w = x(0);
            q.x = x(1);
            q.y = x(2);
            q.z = x(3);

            EulerAngles e;

            e = ToEulerAngles(q);
            
            arma::vec::fixed<3> h;  // measurement prediction
            h(0) = e.roll;
            h(1) = e.pitch;
            h(2) = e.yaw;

            //--------------- Measurement Jacobian
                int x_len = x.size();
                arma::mat H;
                // Form jacobian
                H.resize(3,x_len);
                arma::mat::fixed<3,4> de_dq; 
            double q0 = q.w;
            double q1 = q.x;
            double q2 = q.y;
            double q3 = q.z;
            
            double t1 = (2*q1*q1 + 2*q2*q2 - 1);
            double t2 = (2*q0*q1 + 2*q2*q3);
            double t3 = pow((1 - pow((2*q0*q2 - 2*q1*q3),2)),.5);
            double t4 = (2*q2*q2 + 2*q3*q3 - 1);
            double t5 =  (2*q0*q3 + 2*q1*q2);

            
            de_dq = {{-(2*q1*t1)/(t1*t1 + t2*t2), -(((2*q0)/t1 - (4*q1*t2)/t1*t1)*t1*t1)/(t1*t1 + t2*t2), -(((2*q3)/t1 - (4*q2*t2)/t1*t1)*t1*t1)/(t1*t1 + t2*t2), -(2*q2*t1)/(t1*t1 + t2*t2) },
                    {(2*q2)/t3, -(2*q3)/t3, (2*q0)/t3, -(2*q1)/t3 }, 
                    { -(2*q3*t4)/(t4*t4 + t5*t5), -(2*q2*t4)/(t4*t4 + t5*t5), -(((2*q1)/t4 - (4*q2*t5)/t4*t4)*t4*t4)/(t4*t4 + t5*t5), -(((2*q0)/t4 - (4*q3*t5)/t4*t4)*t4*t4)/(t4*t4 + t5*t5)  }};               
            
            H(arma::span(0,2),arma::span(0,3)) = de_dq;

            //--------------------------------------------------------------------------
            
           
            
            //---- ekf update

            double vs = par.ekf.sigma_att_update*par.ekf.sigma_att_update;  // variance
            arma::mat::fixed<3,3> R;
            R(0,0) = vs;
            R(1,1) = vs;
            R(2,2) = vs*.0000001;


                arma::mat H_P = H*P;

                arma::mat S = H_P*H.t() + R; // Innovation matrix
                    //arma::mat K = P*H.t()*arma::inv_sympd(S);
                arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

                P = P - K*H_P;  // System Covariance matrix update 
                
                arma::vec::fixed<3> inov = z - h;
                double inov_y = inov(2);
                AngleWrap(inov_y);
                inov(2) = inov_y;
                
                x = x + K*(inov);  // System vector update
            
               cout << "z: " << z(2) << " h:" << h(2) <<  " y: " << att.yaw  << endl;  


}
*/
//-------------------------------------------------------------------------------
/*
void Attitude_Update_2(arma::vec& x, arma::mat& P,ATT &att,parameters &par, double &yaw_at_home, CamPose &Init_cam_position )
{

    double measured_yaw = att.yaw; 
    //double p_ini = par.init.roll_init;
    arma::vec::fixed<3> z;  // measurement
    // account for Bebop to camera coordinate transformation
    z(0) = att.roll; 
    z(1) = -att.pitch;
    double yaw_z = -att.yaw - yaw_at_home;
    AngleWrap(yaw_z);
    //z(0) = 0;
    //z(1) = 0;
    z(2) = yaw_z;

    double axis_x = Init_cam_position.axis_x;
    double axis_y = Init_cam_position.axis_y;
    double axis_z = Init_cam_position.axis_z;
    double Ra2b[9];
    Euler_to_Ra2b(axis_x, axis_y, axis_z, Ra2b);
    arma::mat Rr2c(Ra2b,3,3); // convert from arrat to armadillo
    arma::mat::fixed<3,3> Rb =  {{0, -1, 0},
                                 { 1, 0, 0},
                                 { 0, 0, 1}};
    arma::mat::fixed<3,3> Rc2r = Rb*Rr2c;
    
    std::vector<double> quat;
    quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
    double Rn2c_a[9];
    quat2R(&quat[0],Rn2c_a);    
    arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo

    arma::mat::fixed<3,3> Rn2r = Rn2c*Rc2r;

    double roll_r, pitch_r, yaw_r;

    Ra2b_to_Euler(roll_r,pitch_r,yaw_r,&Rn2r[0]);

    //cout << "r: " << roll_r << " p:" << pitch_r <<  " y: " << yaw_r  << endl; 

    arma::vec::fixed<3> h;  // measurement prediction
            h(0) = roll_r;
            h(1) = pitch_r;
            h(2) = yaw_r;

    //--------------- Measurement Jacobian
                int x_len = x.size();
                arma::mat H;
                // Form jacobian
                H.resize(3,x_len);
                //arma::mat::fixed<3,4> de_dq; 
        
        double q0 = quat[0];
        double q1 = quat[1];
        double q2 = quat[2];
        double q3 = quat[3];
        double r11 = Rc2r(0,0);
        double r12 = Rc2r(0,1);
        double r13 = Rc2r(0,2);
        double r21 = Rc2r(1,0);
        double r22 = Rc2r(1,1);
        double r23 = Rc2r(1,2);
        double r31 = Rc2r(2,0);
        double r32 = Rc2r(2,1);
        double r33 = Rc2r(2,2);

        double t1 = (2*q3*r13 + 2*q0*r23 - 2*q1*r33);
        double t2 = (2*q1*r23 - 2*q2*r13 + 2*q0*r33);
        double t3 = (2*q0*q3 + 2*q1*q2);
        double t4 = (2*q0*q1 - 2*q2*q3);
        double t8 = (r23*t3 + r13*t3 - r33*t4);
        double t5 = (2*q0*q3 - 2*q1*q2);
       double t6 = (q0*q0 + q1*q1 - q2*q2 - q3*q3);
       double t7 = (2*q0*q2 + 2*q1*q3);
       double t9 = (r12*t6 - r22*t5 + r32*t7);


        arma::mat::fixed<1,4> droll_dq; 
           
        double sigma = r33*(q0*q0 - q1*q1 - q2*q2 + q3*q3) - r13*(2*q0*q2 - 2*q1*q3) + r23*(2*q0*q1 + 2*q2*q3);           
            
        droll_dq = { -(sigma*sigma*(t1/sigma - (t2*t8)/sigma*sigma))/(t8*t8 + sigma*sigma), (sigma*sigma*(t2/sigma + (t1*t8)/sigma*sigma))/(t8*t8 + sigma*sigma), -(sigma*sigma*((2*q1*r13 + 2*q2*r23 + 2*q3*r33)/sigma + ((2*q0*r13 - 2*q3*r23 + 2*q2*r33)*t8)/sigma*sigma))/(t8*t8 + sigma*sigma), -(sigma*sigma*((2*q0*r13 - 2*q3*r23 + 2*q2*r33)/sigma - ((2*q1*r13 + 2*q2*r23 + 2*q3*r33)*t8)/sigma*sigma))/(pow( (r23*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + r13*t3 - r33*t4),2) + sigma*sigma)};
        
       

       arma::mat::fixed<1,4> dpitch_dq;     
       double sigma2 = r13*t6 - r23*t5 + r33*t7;    
      
 
       double dp_dq0;  
              
       if(r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7 < r23*t5)
       {
         dp_dq0 = (2*q0*r13 - 2*q3*r23 + 2*q2*r33 - (3*pow(-sigma2,1/2)*(2*q0*r13 - 2*q3*r23 + 2*q2*r33))/2)/(pow((sigma2 + pow(-sigma2,3/2)),2) + 1);
       } 
       if(r23*t5 < r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7)
       {
        dp_dq0 =  ((pow(sigma2,3/2) + 2)*(q0*r13 - q3*r23 + q2*r33))/(pow(sigma2,2) + pow(sigma2,3) - 2*pow(sigma2,3/2) + 1);
       } 

      double dp_dq1; 
      if(r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7 < r23*t5)
      {
        dp_dq1=  (2*q1*r13 + 2*q2*r23 + 2*q3*r33 - (3*pow(-sigma2,1/2)*(2*q1*r13 + 2*q2*r23 + 2*q3*r33))/2)/(pow((sigma2 + pow(-sigma2,3/2)),2) + 1);
      }
      if(r23*t5 < r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7)
      {
        dp_dq1 =  ((pow(sigma2,3/2) + 2)*(q1*r13 + q2*r23 + q3*r33))/(pow(sigma2,2) + pow(sigma2,3) - 2*pow(sigma2,3/2) + 1);
      }

     double dp_dq2;
     if(r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7 < r23*t5)
     {
        dp_dq2 = ((3*pow(-sigma2,1/2) - 2)*(q1*r23 - q2*r13 + q0*r33))/(2*pow(-sigma2,5/2) - pow(sigma2,2) + pow(sigma2,3) - 1);
     }
     if(r23*t5 < r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7)
     {
       dp_dq2 =  ((pow(sigma2,3/2) + 2)*(q1*r23 - q2*r13 + q0*r33))/(pow(sigma2,2) + pow(sigma2,3) - 2*pow(sigma2,3/2) + 1);
     } 
     
     double dp_dq3; 
     if(r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7 < r23*t5)
     {
         dp_dq3 = -(2*q3*r13 + 2*q0*r23 - 2*q1*r33 - (3*pow(-sigma2,1/2)*t1)/2)/(pow((sigma2 + pow(-sigma2,3/2)),2) + 1);
     }
     if( r23*t5 < r13*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + r33*t7)
     {
         dp_dq3 = -((pow(sigma2,3/2) + 2)*(q3*r13 + q0*r23 - q1*r33))/(pow(sigma2,2) + pow(sigma2,3) - 2*pow(sigma2,3/2) + 1);
     }

     dpitch_dq = {dp_dq0,dp_dq1,dp_dq2,dp_dq3};

    arma::mat::fixed<1,4> dyaw_dq; 
    double  sigma3 = r11*t6 - r21*t5 + r31*t7;

    dyaw_dq = {-(sigma3*sigma3*((2*q0*r12 - 2*q3*r22 + 2*q2*r32)/sigma3 - ((2*q0*r11 - 2*q3*r21 + 2*q2*r31)*t9)/sigma3*sigma3))/(t9*t9 + sigma3*sigma3), -(sigma3*sigma3*((2*q1*r12 + 2*q2*r22 + 2*q3*r32)/sigma3 - ((2*q1*r11 + 2*q2*r21 + 2*q3*r31)*t9)/sigma3*sigma3))/(t9*t9 + sigma3*sigma3), -(sigma3*sigma3*((2*q1*r22 - 2*q2*r12 + 2*q0*r32)/sigma3 - ((2*q1*r21 - 2*q2*r11 + 2*q0*r31)*t9)/sigma3*sigma3))/(t9*t9 + sigma3*sigma3), (sigma3*sigma3*((2*q3*r12 + 2*q0*r22 - 2*q1*r32)/sigma3 - ((2*q3*r11 + 2*q0*r21 - 2*q1*r31)*t9)/sigma3*sigma3))/(t9*t9 + sigma3*sigma3)};
    
    arma::mat::fixed<3,4> de_dq;
    de_dq.row(0) = droll_dq;
    de_dq.row(1) = dpitch_dq;
    de_dq.row(2) = dyaw_dq;

   H(arma::span(0,2),arma::span(0,3)) = de_dq;
  //  H(0,arma::span(0,3)) = dyaw_dq;
    //---------------------------------------------------------                     
    
    double vs = par.ekf.sigma_att_update*par.ekf.sigma_att_update;  // variance
            arma::mat::fixed<3,3> R;
            R(0,0) = vs*.0001;
            R(1,1) = vs*.0001;
            R(2,2) = vs*.000001;
            
            double r = vs*.0000001;

                arma::mat H_P = H*P;

                arma::mat S = H_P*H.t() + R; // Innovation matrix
                    //arma::mat K = P*H.t()*arma::inv_sympd(S);
                arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

                P = P - K*H_P;  // System Covariance matrix update 
                
           
                

               // double inov_y = yaw_z - yaw_r;
               // AngleWrap(inov_y);
               //x = x + K*(inov_y);  // System vector update
    
               arma::vec::fixed<3> inov = z - h;
                double inov_y = inov(2);
                AngleWrap(inov_y);
                inov(2) = inov_y;

               x = x + K*(inov);  // System vector update 

}

*/

//-------------------------------------------------------------------------------

void Attitude_Update_3(arma::vec& x, arma::mat& P,ATT &att,parameters &par, double &yaw_at_home,  CamPose &Init_cam_position)
{
    
    static double last_qz = 0;

     arma::vec::fixed<4> z;  // quat measurement
    // account for Bebop to camera coordinate transformation
    
    double roll_r = att.roll; 
    double pitch_r = att.pitch;
    double yaw_z = att.yaw + yaw_at_home;  //  use local yaw measurements
    //AngleWrap(yaw_z);

    

    // account for robot to camera coordintate frame 
    double axis_x = pitch_r + Init_cam_position.axis_x;
    double axis_y = -roll_r ;
    double axis_z =  yaw_z ;

    //cout << "axis_x: " << axis_x << " axis_y: " << axis_y << " axis_z:" << axis_z << endl;
        
    double Ra2b[9];
    Euler_to_Ra2b(axis_x, axis_y, axis_z, Ra2b);
    double q_z[4];
    Ra2b_TO_Quat_a2b(Ra2b,q_z);

    if(((q_z[3] > 0)&&(last_qz <0))||((q_z[3] < 0)&&(last_qz >0)))
    {
    // if change of sign in z axis, force update
      x[0] = q_z[0];
      x[1] = q_z[1];  
      x[2] = q_z[2];    
      x[3] = q_z[3];  
    }
    last_qz = q_z[3];
    
    //cout << "q_z: " << q_z[0] << " " << q_z[1] << " " << q_z[2] << " " << q_z[3] << endl;

    //cout << "q_h: " << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << endl;
    
    // vector measurement
    z[0] = q_z[0];
    z[1] = q_z[1];
    z[2] = q_z[2];
    z[3] = q_z[3];
   
    arma::vec::fixed<4> h;  // quat measurement
    h[0] = x[0];
    h[1] = x[1];
    h[2] = x[2];
    h[3] = x[3];

    int x_len = x.size();
    arma::mat H;
    // Form jacobian
    H.resize(4,x_len);
    arma::mat::fixed<4,4> dq_dq = arma::mat(4,4,arma::fill::eye);
    H(arma::span(0,3),arma::span(0,3)) = dq_dq;

    double vs = par.ekf.sigma_att_update*par.ekf.sigma_att_update;  // variance
    arma::mat::fixed<4,4> R;
            R(0,0) = vs; //*.0000001;
            R(1,1) = vs; //*.0000001;
            R(2,2) = vs; //*.0000001;
            R(3,3) = vs; //*.0000001; 
            
            arma::mat H_P = H*P;
            arma::mat S = H_P*H.t() + R; // Innovation matrix
                    
            arma::mat K = P*H.t()*arma::inv(S); // Kalman gain
            P = P - K*H_P;  // System Covariance matrix update     
              
            arma::vec::fixed<4> inov = z - h;                

            x = x + K*(inov);  // System vector update 

}    