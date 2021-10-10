#include "control.h"
#include <unistd.h>
#include "../Transforms/AngleWrap.h"
#include <math.h>
#include <visp3/core/vpConfig.h>


inline bool file_exist(const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

void CONTROL::init_control_loop(LOCKS &locks,bool &running, bool &stop_control,vpRobotBebop2 &drone)
{
    
    
    if(file_exist("control_plan.txt")==false) // if text file doesn't exist, creat it
    {
        std::ofstream file_Control_plan;
        file_Control_plan.open("control_plan.txt");
        file_Control_plan.close();
    }
    
    
    while(running == true)
    {

       if(execute_control_plan == true)
       {  
          ifstream f_c_p;
          f_c_p.open("control_plan.txt");   // open file for reading
            cout << "Executing control plan..." << endl;           
          control_plan(locks,stop_control,f_c_p,drone);
          execute_control_plan = false;
          f_c_p.close();
       }

      
       usleep(10000);

    }
    
    

    

}

//----------------------------------------------------
void CONTROL::set_robot_state(double &x,double &y, double &z, double &yaw)
{
    r_state.x = x;
    r_state.y = y;
    r_state.z = z;
    r_state.yaw = yaw;  
}


//----------------------------------------------------------
void CONTROL::control_plan(LOCKS &locks,bool &stop_control,ifstream &file_Control_plan,vpRobotBebop2 &drone)
{

    
    
    stop_control = false;
    
    
    string cmd_line;
        
        while ( getline(file_Control_plan,cmd_line) && stop_control == false )
        {
            stringstream ss(cmd_line);
            vector<string> cmd_args;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                cmd_args.push_back( substr );
            }
            string cmd = cmd_args[0];

            if(cmd == "")
            {   
                // if there is not another command (blank detected)
                break;
            }

            if(cmd == "pointr") // relative point
            {   
                robot_state r_state_0 = r_state;  // get current state 
                robot_state dp;  // desired state, 
                 dp.x = stod(cmd_args[1]);
                 dp.y = -stod(cmd_args[2]);    
                 dp.z = -stod(cmd_args[3]);    
                 dp.yaw = stod(cmd_args[4])*(3.1416/180);           
                 cout << "-> go to point (r): " << dp.x << " " << -dp.y << " " << -dp.z << " " << dp.yaw  << endl;
                 go_point(true,locks,stop_control, r_state_0, dp,drone );
            }

            if(cmd == "pointa") // absolute point
            {   
                robot_state r_state_0 = r_state;  // get current state  
                robot_state dp;  // desired state, 
                 dp.x = stod(cmd_args[1]);
                 dp.y = -stod(cmd_args[2]);    
                 dp.z = -stod(cmd_args[3]);    
                 dp.yaw = stod(cmd_args[4])*(3.1416/180);           
                 cout << "-> go to point (a): " << dp.x << " " << -dp.y << " " << -dp.z << " " << dp.yaw  << endl;
                 go_point(false,locks,stop_control, r_state_0, dp,drone );
            }

            if(cmd == "takeoff") // absolute point
            {
                cout << "-> Take off" << endl;
                drone.takeOff(true);
            }
            if(cmd == "land") // absolute point
            {
                cout << "-> Landing" << endl;
                drone.land();
            }
            if(cmd == "rechome")
            {                
                recognize_home(locks,stop_control,drone);                
            }
             

            

            if(stop_control == true)
            {   
                drone.stopMoving(); 
                cout << "Stopping control plan.." << endl;
            }    

           //usleep(10000); 
        }



}
//
void CONTROL::recognize_home(LOCKS &locks,bool &stop_control,vpRobotBebop2 &drone)
{
    robot_state r_state_0 = r_state;  // get current state 

    double axis_x = Init_cam_position_axis_x ;
    double axis_y = 0;
    double axis_z = 0;
        
    double Ra2b[9];
    //Euler_to_Ra2b(axis_x, axis_y, axis_z, Ra2b);
    double q_int[4];
    //Ra2b_TO_Quat_a2b(Ra2b,q_int); 




}


//-------------------------------------------------------------------------
void CONTROL::go_point(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone )
{
    // If relative == true, then:
    // the movement is executed relative to its current position.   


    bool arrive = false;
    while(stop_control == false)
    {   
        arma::vec::fixed<4> x;  // current robot state
        x(0) =  r_state.x;
        x(1) =  r_state.y;
        x(2) =  r_state.z;
        x(3) =  r_state.yaw;
        AngleWrap(x(3)); 
        
        arma::vec::fixed<4> xd;  // desired robot state
        if(relative == true)
        {   
            double a = r_state_0.yaw;
            arma::mat::fixed<2,2> Rb2n =  {{cos(a), -sin(a)},
                                           { sin(a), cos(a)}};

            arma::vec::fixed<2> xy_to_go_C;
            xy_to_go_C(0) =  r_state_to_go.x;
            xy_to_go_C(1) =  r_state_to_go.y;                              
           
            arma::vec::fixed<2> xy_to_go_N;
            
            xy_to_go_N = Rb2n*xy_to_go_C;

            xd(0) =  xy_to_go_N(0) + r_state_0.x;
            xd(1) =  xy_to_go_N(1) + r_state_0.y;
            xd(2) =  r_state_to_go.z + r_state_0.z;
            xd(3) =  r_state_to_go.yaw + r_state_0.yaw;
            AngleWrap(xd(3));
        }
        else
        {           
            xd(0) =  r_state_to_go.x;
            xd(1) =  r_state_to_go.y;
            xd(2) =  r_state_to_go.z;
            xd(3) =  r_state_to_go.yaw;
            AngleWrap(xd(3));
        }     

        //cout << "x: " << x(0) << " " << x(1) << " " << x(2) << " " << x(3)  << endl;
        //cout << "xd: " << xd[0] << " " << xd[1] << " " << xd[2] << " " << xd[3]  << endl;

        arma::vec::fixed<4> e = xd - x; // errror
        AngleWrap(e(3)); 

        double norm_xyz_e = sqrt(e(0)*e(0) + e(1)*e(1) + e(2)*e(2));
        double norm_yaw_e = sqrt(e(3)*e(3)); 

        //cout << e << endl;

         if(norm_xyz_e < PAR.control.Max_error_xyz_reach_p && norm_yaw_e < PAR.control.Max_error_yaw_reach_p )
         {   
             drone.stopMoving(); 
             // if error is less than some threshold break
             cout << "-> point reached" << endl;
             break;
         }

        // set gains
        double kx,ky,kz,kyaw;
        kx= PAR.control.kx;
        ky = PAR.control.ky;
        kz = PAR.control.kz;
        kyaw = PAR.control.kyaw;

        vpColVector ve(4);  // velocity vector

        double vx = -kx*e(0);
        double vy = -kx*e(1);
        double vz = -kx*e(2);
        double vyaw = -kx*e(3);

        // check for max velocities allowed
        if(vx > PAR.control.MaxVel_xy ) vx = PAR.control.MaxVel_xy;
        if(vx < -PAR.control.MaxVel_xy ) vx = -PAR.control.MaxVel_xy;
        if(vy > PAR.control.MaxVel_xy ) vy = PAR.control.MaxVel_xy;
        if(vy < -PAR.control.MaxVel_xy ) vy = -PAR.control.MaxVel_xy;
        if(vz > PAR.control.MaxVel_xy ) vx = PAR.control.MaxVel_z;
        if(vz < -PAR.control.MaxVel_xy ) vx = -PAR.control.MaxVel_z;
        if(vyaw > PAR.control.MaxVel_xy ) vyaw = PAR.control.MaxVel_yaw;
        if(vyaw < -PAR.control.MaxVel_xy ) vyaw = -PAR.control.MaxVel_yaw;

       
        // transform velocities to bebop coodinate frame

        double a = x(3) + (3.1416/2) ;  // current yaw   
        arma::mat::fixed<2,2> Rb2n =  {{cos(a), -sin(a)},
                                           { sin(a), cos(a)}};
        arma::vec::fixed<2> v_xy_N;
        v_xy_N(0) =  vx;
        v_xy_N(1) =  vy;                              
           
       arma::vec::fixed<2> v_xy_R = Rb2n.t()*v_xy_N;

        //ve[0] = vy;
        //ve[1] = -vx;
        ve[0] = v_xy_R(0);
        ve[1] = v_xy_R(1);
        
        ve[2] = -vz;
        ve[3] = -vyaw;
        
        //drone.setVelocity(ve, 1.0);
       // cout << "e: " << e(0) << " " << e(1) << " " << e(2) << " " << e(3)  << endl;
        
        cout << "ve_n: " << vx << " " << vy << " " << vz << " " << vyaw  << endl;

        cout << "ve_r: " << ve[0] << " " << ve[1] << " " << ve[2] << " " << ve[3]  << endl;

        usleep(40000);  // wait 40 milliseconds 
    }    
    drone.stopMoving(); 
  



}



