#include "control.h"
#include <unistd.h>
#include "../Transforms/AngleWrap.h"
#include "../Transforms/Euler_to_Ra2b.h"
#include "../Transforms/Ra2b_TO_Quat_a2b.h"
#include "../vision/vision.h"
#include <math.h>
#include <visp3/core/vpConfig.h>
#include "../matplotlib/matplotlibcpp.h"
namespace plt = matplotlibcpp;


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
    int ne_cmd = 0;
        
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
                 string break_info;
                 go_point(true,locks,stop_control, r_state_0, dp,drone,false,break_info );
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
                 string break_info;
                 go_point(false,locks,stop_control, r_state_0, dp,drone,false,break_info  );
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
            if(cmd == "home")
            {
                home(locks,stop_control,drone); 
            }            

            if(stop_control == true)
            {   
                drone.stopMoving(); 
                cout << "Stopping control plan.." << endl;
            }    

           //usleep(10000);

           ne_cmd++; 
        }

        cout << "control plan finished, " << ne_cmd << " commands executed" << endl;

}
// ----------------------------------------------------------------------------------
void CONTROL::home(LOCKS &locks,bool &stop_control,vpRobotBebop2 &drone)
{   
    robot_state r_state_0 = r_state;  // get current state  
    loop_closed = false;
    robot_state ph;  
    ph.x = 0;
    ph.y = 0;
    ph.z = r_state_0.z;
    ph.yaw = 0;  
    // go back to home
    string break_info;
    cout << "-> going to home" << endl;
    go_point(false,locks,stop_control, r_state_0, ph,drone,true,break_info );  // absolute points

    if( break_info == "loop_closed")
    {
       go_point(false,locks,stop_control, r_state_0, ph,drone,false,break_info );  // absolute points
       cout << "-> Home reached" << endl;
    }
    else
    {  
       double lambda_y = PAR.control.home_lamba;   // ----------
       double y_a = PAR.control.home_y_a_def; 
       double x_a = PAR.control.home_x_a_def;    
       cout << "-> Exploring home area from: (x: " << x_a << " y: " << y_a << ")   to (x: " << -x_a  << " y: " << -y_a << ")"<< endl; 
       string break_infoE;
       explore_area(locks,stop_control,lambda_y, x_a , y_a, drone,true,break_infoE);

       if( break_infoE == "loop_closed")
        {
            go_point(false,locks,stop_control, r_state_0, ph,drone,false,break_info );  // absolute points
            cout << "-> Home reached" << endl;
        }
       else
       {   
           go_point(false,locks,stop_control, r_state_0, ph,drone,false,break_info );  // absolute points
           cout << "-> A loop has not been detected!" << endl; 
       }
    }    




}
// ----------------------------------------------------------------------------------
void CONTROL::recognize_home(LOCKS &locks,bool &stop_control,vpRobotBebop2 &drone)
{
    robot_state r_state_0 = r_state;  // get current state  
    double axis_x = Init_cam_position_axis_x ;
    double axis_y = 0;
    double axis_z = r_state.yaw;        
    double Ra2b[9];
    Euler_to_Ra2b(axis_x, axis_y, axis_z, Ra2b);    
    cv::Point2d uvd;
    uvd.x = PAR.img.image_cols;  //320
    uvd.y = PAR.img.image_rows;  //240
    arma::vec::fixed<3> hc;
    arma::mat::fixed<3,2> dhc_duvd;
    hc = Inverse_projection_model(uvd,1,false, PAR.img.cam_parameters,dhc_duvd); // compute inverse projection model    
    arma::mat Rn2c(Ra2b,3,3); // convert from arrat to armadillo
    arma::mat::fixed<3,3> Rc2n = Rn2c.t();
    arma::vec::fixed<3> hn = Rc2n*hc;    
    arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature
    arma::vec::fixed<3> v = {0,0,1};
    arma::vec::fixed<3> vn =  Rc2n*v;    
    double depth = (abs(r_state.z)) /arma::dot(m,vn);    
    arma::vec::fixed<3> p_r =  depth*m;    
    double yv = p_r(1);
    double lambda_y = 0;   // ----------
    double ym = yv + yv*lambda_y;    
  
    double y_a = ym; 
    double x_a = ym;    
    
    cout << "-> Exploring home area from: (x: " << x_a << " y: " << y_a << ")   to (x: " << -x_a  << " y: " << -y_a << ")"<< endl; 
    
    string break_info;
    explore_area(locks,stop_control,lambda_y, x_a , y_a, drone,false,break_info);

    robot_state ph;  
    ph.x = r_state_0.x;
    ph.y = r_state_0.y;
    ph.z = r_state_0.z;
    ph.yaw = r_state_0.yaw;  
    // go back to home
    string break_info2;
    go_point(false,locks,stop_control, r_state_0, ph,drone,false,break_info2  );  // absolute points

    cout << "-> Exploring home area done " << endl;


}

//-----------------------------------------------------------------
bool CONTROL::explore_area(LOCKS &locks,bool &stop_control, double lambda_y, double x_a, double y_a, vpRobotBebop2 &drone, bool break_at_CL, std::string &break_info)
{
    // lambda_y [-1, 1] indicates the minimun overlap region along y axis

    robot_state r_state_0 = r_state;  // get current state 
 
    double axis_x = Init_cam_position_axis_x ;
    double axis_y = 0;
    double axis_z = r_state.yaw;
        
    double Ra2b[9];
    Euler_to_Ra2b(axis_x, axis_y, axis_z, Ra2b);    
    cv::Point2d uvd;
    uvd.x = PAR.img.image_cols;  //320
    uvd.y = PAR.img.image_rows;  //240
    arma::vec::fixed<3> hc;
    arma::mat::fixed<3,2> dhc_duvd;
    hc = Inverse_projection_model(uvd,1,false, PAR.img.cam_parameters,dhc_duvd); // compute inverse projection model    
    arma::mat Rn2c(Ra2b,3,3); // convert from arrat to armadillo
    arma::mat::fixed<3,3> Rc2n = Rn2c.t();
    arma::vec::fixed<3> hn = Rc2n*hc;    
    arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature
    arma::vec::fixed<3> v = {0,0,1};
    arma::vec::fixed<3> vn =  Rc2n*v;
    
    double depth = (abs(r_state.z)) /arma::dot(m,vn);
    //double depth = (1) /arma::dot(m,vn);
    
    arma::vec::fixed<3> p_r =  depth*m; 
    
    double xv = p_r(0);
    double yv = p_r(1);
    double zv = p_r(2);

    
    double ym = yv + yv*lambda_y;
    int n_seg_y = int(y_a/ym);
    if (n_seg_y == 0) n_seg_y = 1; 

    double dy = y_a/(double)n_seg_y;

    double py = y_a;
    
    vector<robot_state> pr;
    vector<double> x_i,y_i;
    
    
    // initial point
    robot_state p;  
     p.x = x_a + r_state_0.x;
     p.y = py + r_state_0.y;
      p.z = r_state_0.z;
      p.yaw = r_state_0.yaw;      
      pr.push_back(p); 
    //-----------------------  
      x_i.push_back(p.x);
      y_i.push_back(-p.y);
      
      bool right = 0;  
      int c_lr = 0;
    
    for(int i = 1; i < (n_seg_y*4 +2)  ; i++)
    {      
      robot_state p;      
      if(right == 1)
      {
          p.x = x_a + r_state_0.x;
          p.y = py + r_state_0.y;
          c_lr ++;
            if(c_lr == 2) 
            {
                right = 0;
                c_lr = 0;
            }
            else
            {
                py = py - dy;
            }     
      }
      else
      {
         p.x = -x_a + r_state_0.x; 
         p.y = py + r_state_0.y;
         
         c_lr ++;
         if(c_lr == 2) 
         {
             right = 1;
             c_lr = 0;
         }
         else
         {
            py = py - dy;
         }    

      }      
      
      p.z = r_state_0.z;
      p.yaw = r_state_0.yaw;

      pr.push_back(p);            
      
      x_i.push_back(p.x);
      y_i.push_back(-p.y);

    }

   // plt::figure(1); 
   // plt::plot(x_i,y_i,"y."); 
   // plt::axis("equal");    
   // plt::show();      
    
    for(int i=0; i < pr.size(); i++)
    {   
        robot_state r_state_0 = r_state;  // get current state 
        string break_infoP;
        if (break_at_CL == false)
        { 
         go_point(false,locks,stop_control, r_state_0, pr[i],drone ,false,break_info );  // absolute points
         cout << "-> go to point (a): " << pr[i].x << " " << -pr[i].y << " " << -pr[i].z << " " << pr[i].yaw  << endl;
        }
        else
        {
          go_point(false,locks,stop_control, r_state_0, pr[i],drone ,true,break_infoP );  // absolute points 

          if (break_infoP == "loop_closed")
          {
             cout << "-> Loop detected while exploring area, stoping exploration" << endl;
             break_info = "loop_closed";
             return false;
          }

        } 
         
    }

    return true;    

}

/*
// p1
    robot_state p1;
    p1.x = xv + xv*lambda_x;
    p1.y = 0;
    p1.z = 0;
    p1.yaw = 0;
    pr.push_back(p1);
    //
    robot_state p2;
    p2.x = 0;
    p2.y = -(yv + yv*lambda_y);
    p2.z = 0;
    p2.yaw = 0;
    pr.push_back(p2);
    //
    robot_state p3;
    p3.x = -2*(xv + xv*lambda_x);
    p3.y = 0;
    p3.z = 0;
    p3.yaw = 0;
    pr.push_back(p3);
    //
    robot_state p4;
    p4.x = 0;
    p4.y = 2*(yv + yv*lambda_y);
    p4.z = 0;
    p4.yaw = 0;
    pr.push_back(p4);
    //
    robot_state p5;
    p5.x = 2*(xv + xv*lambda_x);
    p5.y = 0;
    p5.z = 0;
    p5.yaw = 0;
    pr.push_back(p5);

*/

//-------------------------------------------------------------------------
bool CONTROL::go_point(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone , bool break_at_CL, std::string &break_info)
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
             return true;
             //break;
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
        
        drone.setVelocity(ve, 1.0);
       // cout << "e: " << e(0) << " " << e(1) << " " << e(2) << " " << e(3)  << endl;
        
        //cout << "ve_n: " << vx << " " << vy << " " << vz << " " << vyaw  << endl;

        //cout << "ve_r: " << ve[0] << " " << ve[1] << " " << ve[2] << " " << ve[3]  << endl;

       if(break_at_CL == true) 
       {   
           // only if flag break_at_CL is set, stop and return with false if a close loop is detected
           if(loop_closed == true) // 
           {
               drone.stopMoving();
               break_info = "loop_closed";
               return false; 
           }


       }


        usleep(40000);  // wait 40 milliseconds 
    }    
    drone.stopMoving();
    break_info = "stop_control";
    return false; 
  



}



