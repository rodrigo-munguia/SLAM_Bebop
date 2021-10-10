#ifndef CONTROL_H
#define CONTROL_H

#include "../parameters.h"
#include "../locks.h"
#include <iostream>
#include <fstream>
#include <armadillo>
#include <sstream>
#include "../Bebop2.h"



struct robot_state
{
    double x;
    double y;
    double z;
    double yaw;
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
       
        void set_robot_state(double &x,double &y, double &z, double &yaw);

        double Init_cam_position_axis_x;

    private:
        parameters PAR;

        robot_state r_state; 

        void control_plan(LOCKS &locks,bool &stop_control,ifstream &file_Control_plan,vpRobotBebop2 &drone); 
        
        void go_point(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_at_home, robot_state r_state_to_go,vpRobotBebop2 &drone );

        void recognize_home(LOCKS &locks,bool &stop_control,vpRobotBebop2 &drone);

    
};








#endif