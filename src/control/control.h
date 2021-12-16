#ifndef CONTROL_H
#define CONTROL_H

#include "../parameters.h"
#include "../locks.h"
#include <iostream>
#include <fstream>
#include <armadillo>
#include <sstream>
#include "../Bebop2.h"




struct gainLQR
{
   double k1;
   double k2;
   double k3;
};


struct gain
{
    double x;
    double y;
    double z;
    double yaw;
};


struct robot_state
{
    double x;
    double y;
    double z;
    double yaw;
    double vx;
    double vy;
    double vz;
};

using namespace std;

class CONTROL
{

    public:       
        
        CONTROL(parameters &par) // cosntructor
        {
            PAR = par;        
            
        }

        void init_control_loop(LOCKS &locks,bool &running, bool &stop_control,vpRobotBebop2 &drone);    
       
        bool execute_control_plan = false;
       
        void set_robot_state(double &x,double &y, double &z, double &yaw, double &vx, double &vy, double &vz);

        double Init_cam_position_axis_x;

        bool loop_closed = false;

    private:
        parameters PAR;

        robot_state r_state; 

        arma::mat::fixed<2,2> Pc;
        bool compute_uncertanty_flag = false;

        void exploration_uncertainty(bool init,bool reset);
        void compute_uncertanty(double vx, double vy, double dt);

        void control_plan(LOCKS &locks,bool &stop_control,ifstream &file_Control_plan,vpRobotBebop2 &drone); 
        
        //bool go_point(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info);
        //bool go_point2(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info);
        //arma::vec::fixed<4> PI_control(arma::vec::fixed<4> xd, arma::vec::fixed<4> x, bool flag, gain &Kp, gain &Ki , double Ts);
        
        void recognize_home(LOCKS &locks,bool &stop_control,vpRobotBebop2 &drone);

        bool explore_area(LOCKS &locks,bool &stop_control, double lambda_y, double x_a, double y_a, vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info);

        void home(LOCKS &locks,bool &stop_control,vpRobotBebop2 &drone);        
        
        //----
        bool go_point_xy(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info);
        bool go_point_xy_inter(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info,bool final);
        arma::vec::fixed<2> LQR_control_xy(arma::vec::fixed<2> xd, arma::vec::fixed<4> x, bool flag, gainLQR &Kx, gainLQR &Ky,  double Ts);
        //---
        bool go_alt(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info);
        double LQR_control_alt(double xd, arma::vec::fixed<2> x, bool flag, gainLQR &Kz,  double Ts);
        //---
        bool go_yaw(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info);
        double control_yaw(double xd, double x, bool flag, double Kp, double Ki,  double Ts);


};








#endif