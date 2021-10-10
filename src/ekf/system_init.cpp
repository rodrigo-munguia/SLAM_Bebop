
#include "system_init.h"


//-----------------------------------------------------------------------------
//   State vector and Covariance matrix initialization
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

/*
% Initial States ***************************************************
% x meaning
% index  0  1  2  3  4   5   6   7  8  9  10  11  12  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(1:4)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to camera)
% x(5:7)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(8:10)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(11:13)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.
*/

bool system_init(arma::vec& x, arma::mat& P, parameters &par, DATA &dat,double &yaw_at_home,CamPose &Init_cam_position)
{
    static int n_att = 0;
    static double sum_yaw = 0;
    int min_n_dat = 10;

    double z_yaw = -dat.att.yaw; // invert yaw measurement

    if(n_att >= min_n_dat)        
    {   
        cout << endl ; 
        double sy = sum_yaw;
        
        yaw_at_home = sy/(double)min_n_dat;  

        n_att = 0;
        sum_yaw = 0;

        x.resize(13);
        P.resize(13,13);

        double axis_x = Init_cam_position.axis_x ;
        double axis_y = Init_cam_position.axis_y ;
        double axis_z =  Init_cam_position.axis_z ;
        double x_init = Init_cam_position.x;
        double y_init = Init_cam_position.y;
        double z_init = Init_cam_position.z;
        double Ra2b[9];

        Euler_to_Ra2b(axis_x, axis_y, axis_z, Ra2b);

        double q_int[4];

        Ra2b_TO_Quat_a2b(Ra2b,q_int);

        x(0) = q_int[0];
        x(1) = q_int[1];
        x(2) = q_int[2];
        x(3) = q_int[3];
        x(4) = numeric_limits<double>::epsilon();
        x(5) = numeric_limits<double>::epsilon();
        x(6) = numeric_limits<double>::epsilon();
        x(7) = x_init;
        x(8) = y_init;
        x(9) = z_init;
        x(10) = numeric_limits<double>::epsilon();
        x(11) = numeric_limits<double>::epsilon();
        x(12) = numeric_limits<double>::epsilon();

        P = eye(13,13)*numeric_limits<double>::epsilon();

        return true;
    }   

    if(dat.data_type == "attitude")
    {
       n_att++;
       sum_yaw = sum_yaw + z_yaw;
       cout << "." ;      

       return false;
    }
    else
    {
       return false;
    } 




}