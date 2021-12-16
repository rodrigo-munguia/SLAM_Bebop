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
void CONTROL::set_robot_state(double &x,double &y, double &z, double &yaw, double &vx, double &vy, double &vz)
{
    r_state.x = x;
    r_state.y = y;
    r_state.z = z;
    r_state.yaw = yaw; 
    r_state.vx = vx;
    r_state.vy = vy;
    r_state.vz = vz; 
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
                // dp.z = -stod(cmd_args[3]);    
                // dp.yaw = stod(cmd_args[4])*(3.1416/180);           
                 cout << "-> go to point (r): " << dp.x << " " << -dp.y  << endl;
                 string break_info;
                 go_point_xy(true,locks,stop_control, r_state_0, dp,drone,false,break_info );
            }

            if(cmd == "pointa") // absolute point
            {   
                robot_state r_state_0 = r_state;  // get current state  
                robot_state dp;  // desired state, 
                 dp.x = stod(cmd_args[1]);
                 dp.y = -stod(cmd_args[2]);    
                 //dp.z = -stod(cmd_args[3]);    
                 //dp.yaw = stod(cmd_args[4])*(3.1416/180);           
                 cout << "-> go to point (a): " << dp.x << " " << -dp.y  << endl;
                 string break_info;
                 go_point_xy(false,locks,stop_control, r_state_0, dp,drone,false,break_info  );
            }
            if(cmd == "alta")
            {
                robot_state r_state_0 = r_state;  // get current state  
                robot_state dp;  // desired state,
                dp.z = -stod(cmd_args[1]);
                cout << "-> Set altitude to: " << -dp.z << " m"<< endl;
                string break_info; 
                go_alt(false,locks,stop_control, r_state_0, dp,drone,false,break_info);
            }
            if(cmd == "yawa")
            {
                robot_state r_state_0 = r_state;  // get current state  
                robot_state dp;  // desired state,
                dp.yaw = stod(cmd_args[1])*(3.1416/180); 
                cout << "-> Set yaw to: " << cmd_args[1] << " deg"<< endl;
                string break_info; 
                go_yaw(false,locks,stop_control, r_state_0, dp,drone,false,break_info);
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
            if(cmd == "startex")
            {   
                cout << "-> Starting exploration" << endl;
                exploration_uncertainty(true,true);

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
//-----------------------------------------------------------------------------------
void CONTROL::exploration_uncertainty(bool init,bool reset)
{

   if (init == true)
   {   
    compute_uncertanty_flag = true;
   }
   else
   {
     compute_uncertanty_flag = false;  
   }
   if (reset == true)
   { 
        Pc = { {0, 0},                       
               {0, 0} };
   }
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
    go_point_xy(false,locks,stop_control, r_state_0, ph,drone,true,break_info );  // absolute points

    if( break_info == "loop_closed")
    {  
       r_state_0 = r_state;  // get current state   
       go_point_xy(false,locks,stop_control, r_state_0, ph,drone,false,break_info );  // absolute points
       cout << "-> Home reached!!!" << endl;
    }
    else
    {  
       double lambda_y = PAR.control.home_lamba;   // ----------
       
       double sigma_x =  sqrt(Pc(0,0));    // get uncertanty from compute uncertanty function
       double sigma_y =  sqrt(Pc(1,1));
       
       double x_a, y_a;
       if (sigma_x > 0 && sigma_x < PAR.control.home_x_max)
       {
          x_a = sigma_x; 
          y_a = sigma_y;   
       }
       else
       {
         y_a = PAR.control.home_y_max; 
         x_a = PAR.control.home_x_max;
       }      
       
       cout << "-> Exploring home area from: (x: " << x_a << " y: " << y_a << ")   to (x: " << -x_a  << " y: " << -y_a << ")"<< endl; 
       
       string break_infoE;
       explore_area(locks,stop_control,lambda_y, x_a , y_a, drone,true,break_infoE);

       

       if( break_infoE == "loop_closed")
        {   
            r_state_0 = r_state;  // get current state  
            go_point_xy(false,locks,stop_control, r_state_0, ph,drone,false,break_info );  // absolute points
            cout << "-> Home reached" << endl;
        }
       else
       {   
           // last try to detect the home
           r_state_0 = r_state;  // get current state  
           go_point_xy(false,locks,stop_control, r_state_0, ph,drone,true,break_info );  // absolute points

            if( break_info == "loop_closed")
            {   
                r_state_0 = r_state;  // get current state  
                go_point_xy(false,locks,stop_control, r_state_0, ph,drone,false,break_info );  // absolute points
                cout << "-> Home reached!!" << endl;
            }
            else
            {
                cout << "-> The home has not been detected" << endl;
            }
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
    double lambda_y = -.25;   // ----------
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
    r_state_0 = r_state;  // get current state  
    go_point_xy(false,locks,stop_control, r_state_0, ph,drone,false,break_info2  );  // absolute points

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
         cout << "-> go to point (a): " << pr[i].x << " " << -pr[i].y  << endl;    
         go_point_xy(false,locks,stop_control, r_state_0, pr[i],drone ,false,break_info );  // absolute points
         
        }
        else
        { 
          cout << "-> go to point (a): " << pr[i].x << " " << -pr[i].y  << endl;
          go_point_xy(false,locks,stop_control, r_state_0, pr[i],drone ,true,break_infoP );  // absolute points 

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

//------------------------------------------------------------------------
double sat(double in,double limit)
{   
    double out;
    out = in;
    if (in > limit) out = limit;
    if (in < -limit) out = -limit;    

    return out;
}
//---------------------------------------------------------------------------------------------
void CONTROL::compute_uncertanty(double vx, double vy, double dt)
{

    arma::mat::fixed<2,2>  Q; 
     Q ={ {pow(PAR.control.sigma_x,2), 0},                       
          {0, pow(PAR.control.sigma_y,2)} };

     Pc = Pc + Q*dt;
     
    //double sigma_x =  sqrt(Pc(0,0));    
    //double sigma_y =  sqrt(Pc(1,1));
    //cout << "Sigma_x: " << sigma_x << "Sigma_y: " << sigma_y <<  endl;

}

//---------------------------------------------------------------------------------------------
arma::vec::fixed<2> CONTROL::LQR_control_xy(arma::vec::fixed<2> xd, arma::vec::fixed<4> x, bool flag, gainLQR &Kx, gainLQR &Ky,  double Ts)
{
    static  double xi_x;
    static  double xi_y;    
    static double x_0;
    static double y_0;    

    if(flag == true)
    {
        xi_x = 0;
        xi_y = 0;        
        x_0 = x(0);
        y_0 = x(1);        
    }
    
    double r_x = xd(0);
    double r_y = xd(1);
    
    double px = x(0);
    double vx = x(2);
    double py = x(1);
    double vy = x(3);    

    xi_x = xi_x + (-px + (r_x-x_0))*Ts;
    xi_y = xi_y + (-py + (r_y-y_0))*Ts;    
    
    arma::vec::fixed<2> u_unsat;    
   // u_unsat(0) = (-Kx.k1*vx -Kx.k2*(px-x_0) -Kx.k3*xi_x)*(180/3.14);
   // u_unsat(1) = (-Ky.k1*vy -Ky.k2*(py-y_0) -Ky.k3*xi_y)*(180/3.14);
    
    u_unsat(0) = (-Kx.k1*vx -Kx.k2*(px-r_x))*(180/3.14);
    u_unsat(1) = (-Ky.k1*vy -Ky.k2*(py-r_y))*(180/3.14);   

    arma::vec::fixed<2> u;
    u(0) = sat(u_unsat(0),PAR.control.MaxTilt);
    u(1) = sat(u_unsat(1),PAR.control.MaxTilt);   

    // integrator anti-windup
    xi_x = xi_x + (Ts/Kx.k3)*(u(0) - u_unsat(0));
    xi_y = xi_y + (Ts/Ky.k3)*(u(1) - u_unsat(1));  

    if(compute_uncertanty_flag == true)
    {
        compute_uncertanty(vx,vy,Ts);
    } 

    return u;
}   
//-------------------------------------------------------------------------------------------
double CONTROL::LQR_control_alt(double xd, arma::vec::fixed<2> x, bool flag, gainLQR &Kz,  double Ts)
{
    static  double xi_z;  
    static double z_0;      

    if(flag == true)
    {
        xi_z = 0;               
        z_0 = x(0);             
    }
    
    double r_z = xd;  
    
    double pz = x(0);
    double vz = x(1);       

    xi_z = xi_z + (-pz + (r_z-z_0))*Ts;
    
    
    double u_unsat;    
   // u_unsat = (-Kz.k1*vz -Kz.k2*(pz-z_0) -Kz.k3*xi_z)*(180/3.14);  
    
    u_unsat = -Kz.k1*vz -Kz.k2*(pz-r_z);   

    double u;
    u = sat(u_unsat,PAR.control.MaxVel_z2);    

    // integrator anti-windup
    xi_z = xi_z + (Ts/Kz.k3)*(u - u_unsat);

    return u;

}
//------------------------------------------------------------------------------------------
double CONTROL::control_yaw(double xd, double x, bool flag, double Kp, double Ki,  double Ts)
{
    static  double integrator_e;
    static  double error_d1;

    if(flag == true)
    {
        integrator_e = 0;       
        error_d1 = 0;        
    }
    
    double error = xd - x; // error
    AngleWrap(error);

    integrator_e = integrator_e + (Ts/2)*(error + error_d1);
    error_d1 = error;

    double u_unsat;
    if (Ki == 0)
    {
       u_unsat = Kp*error; 
    }
    else
    {
        u_unsat = Kp*error + Ki*integrator_e;
    }

    double u;    
    u = sat(u_unsat,PAR.control.MaxVel_yaw2);

    // integrator anti-windup    
    integrator_e = integrator_e + (Ts/Ki)*(u - u_unsat);

   return u;

}

//-------------------------------------------------------------------------------------------
bool CONTROL::go_yaw(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone , bool break_at_CL, std::string &break_info)
{ 
    bool arrive = false;
    bool init_c = false;
    

    while(stop_control == false)
    {   
        double yaw;  // current robot state
        yaw =  r_state.yaw;                           
        
        double xd;  // desired robot state
        if(relative == true)
        {               
            xd =  r_state_to_go.yaw + r_state_0.yaw;      
        }
        else
        {           
            xd =  r_state_to_go.yaw;           
        }                           
       
        double e = xd - yaw; // error
        double norm_e = sqrt(e*e);              

        if(norm_e < PAR.control.Max_error_yaw_reach_p)
        {   
             drone.stopMoving(); 
             // if error is less than some threshold break
             cout << "-> yaw reached" << endl;
             return true;             
         }
        
        double Kp = 100;
        double Ki = 0;         
        
        double u;
        if(init_c == false)
        {
            u = control_yaw(xd, yaw, true, Kp,Ki ,.04);
            init_c = true;
        }
        else
        {
            u = control_yaw(xd, yaw, false, Kp,Ki ,.04); 
        }

       cout << "u: " << u << endl; 
       
       drone.setYawSpeed(u); //  + Roll right           

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
       usleep(40000);  // wait 50 milliseconds 

    }
    drone.stopMoving();
    break_info = "stop_control";
    return false; 



}
//---------------------------------------------------------------------------------------------
bool CONTROL::go_alt(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone , bool break_at_CL, std::string &break_info)
{ 
    bool arrive = false;
    bool init_c = false;
    

    while(stop_control == false)
    {   
        arma::vec::fixed<2> x;  // current robot state
        x(0) =  r_state.z;
        x(1) =  r_state.vz;                    
        
        double xd;  // desired robot state
        if(relative == true)
        {               
            xd =  r_state_to_go.z + r_state_0.z;      
        }
        else
        {           
            xd =  r_state_to_go.z;           
        }     
        
        double p;  // current robot state
        p =  x(0);        
       
        double e = xd - p; // error
        double norm_e = sqrt(e*e);              

        if(norm_e < PAR.control.Max_error_z_reach_p )
        {   
             drone.stopMoving(); 
             // if error is less than some threshold break
             cout << "-> alt reached" << endl;
             return true;             
         }

        gainLQR Kz;
        Kz.k1 = 100;
        Kz.k2 = 200;
        Kz.k3 = -20;    
        
        double u;
        if(init_c == false)
        {
            u = LQR_control_alt(xd, x, true, Kz ,.04);
            init_c = true;
        }
        else
        {
            u = LQR_control_alt(xd, x, false, Kz ,.04); 
        }

       //cout << "u: " << -u << endl;          
              
       drone.setVerticalSpeed(-u);  // + Up 
           
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
       usleep(40000);  // wait 50 milliseconds 

    }
    drone.stopMoving();
    break_info = "stop_control";
    return false; 
}    

//-------------------------------------------------------------------------------------------
bool CONTROL::go_point_xy_inter(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone , bool break_at_CL, std::string &break_info,bool final)
{
    // LQR control
    bool arrive = false;
    bool init_c = false;

    while(stop_control == false)
    {   
        arma::vec::fixed<4> x;  // current robot state
        x(0) =  r_state.x;
        x(1) =  r_state.y;        
        x(2) =  r_state.vx;
        x(3) =  r_state.vy;         
        
        arma::vec::fixed<2> xd;  // desired robot state
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
           
        }
        else
        {           
            xd(0) =  r_state_to_go.x;
            xd(1) =  r_state_to_go.y;
            
        }     
        arma::vec::fixed<2> p;  // current robot state
        p(0) =  x(0);
        p(1) =  x(1);
       
        arma::vec::fixed<2> e = xd - p; // error
        double norm_xy_e = sqrt(e(0)*e(0) + e(1)*e(1));              

        if(norm_xy_e < PAR.control.Max_error_xy_reach_p )
        {   
            if(final == true)
               { 
                 // if it is a final point then stop the drone  
                  drone.stopMoving();
               }  
             // if error is less than some threshold break
             cout << "-> point reached" << endl;
             return true;             
         }

        gainLQR Kx;
        Kx.k1 = .3;
        Kx.k2 = .6;
        Kx.k3 = -.2;

        gainLQR Ky;
        Ky.k1 = .3;
        Ky.k2 = .6;
        Ky.k3 = -.2;
        
        arma::vec::fixed<2> u;
        if(init_c == false)
        {
            u = LQR_control_xy(xd, x, true, Kx, Ky ,.04);
            init_c = true;
        }
        else
        {
           u = LQR_control_xy(xd, x, false, Kx, Ky ,.04); 
        }

       // transform velocities to bebop coodinate frame
        double a = r_state.yaw + (3.1416/2) ;  // current yaw   
        arma::mat::fixed<2,2> Rb2n =  {{cos(a), -sin(a)},
                                           { sin(a), cos(a)}};
        arma::vec::fixed<2> u_xy_N;
        u_xy_N(0) =  u(0);
        u_xy_N(1) =  u(1);                              
           
        arma::vec::fixed<2> u_xy_R = Rb2n.t()*u_xy_N;

        arma::vec::fixed<2> u_b;
        u_b(0) = -u_xy_R(0);
        u_b(1) = -u_xy_R(1);        
           
       //cout << xd; 
       //cout << "u: " << u(0) << " " << u(1)  << endl;
       //cout << "u_b: " << u_b(0) << " " << u_b(1)   << endl;
       
       double p_pitch = u_b(0)*(100/PAR.control.MaxTilt);
       double p_roll = u_b(1)*(100/PAR.control.MaxTilt);

       drone.setPitch(p_pitch); // + Forward
       drone.setRoll(p_roll);  //+ Right   

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
       usleep(40000);  // wait 50 milliseconds 

    }
    drone.stopMoving();
    break_info = "stop_control";
    return false; 
}    
//-------------------------------------------------------------------------------------------
bool CONTROL::go_point_xy(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone , bool break_at_CL, std::string &break_info)
{ 
    
   arma::vec::fixed<2> go_vec; // total displacement vector (relative movement expressed)
   if (relative == true) // relative positions
    {
        go_vec(0) = r_state_to_go.x ;
        go_vec(1) = r_state_to_go.y ;
    }
   else
    {    
        go_vec(0) = r_state_to_go.x - r_state_0.x;
        go_vec(1) = r_state_to_go.y - r_state_0.y;
    }    

   //cout << go_vec << endl;
   //cout << r_state_0.x << " " << r_state_0.y << endl; 

   double norm_go_vec = sqrt(go_vec(0)*go_vec(0)+go_vec(1)*go_vec(1)); // norm (lenght) of go_vec
   
   double s_v = PAR.control.intermediate_point_length_xy;

   if (norm_go_vec > s_v)
   {
       int n_seg = norm_go_vec/s_v + 1;
       double dx = go_vec(0)/n_seg;  
       double dy = go_vec(1)/n_seg;
       double Dx = dx;
       double Dy = dy; 
       //cout << dx << " " << dy << endl; 
       robot_state r_state_to_go_seg;
       r_state_to_go_seg.x = 0;
       r_state_to_go_seg.y = 0;

       for (int i = 0; i < n_seg-1 ; i++) // for each segment of the trajectory
       {
          
          if (relative == true) // relative positions
          {
            r_state_to_go_seg.x = Dx;
            r_state_to_go_seg.y = Dy;
            Dx = Dx + dx;
            Dy = Dy + dy;             
            cout << "-> I-point (r): " << r_state_to_go_seg.x << " " << -r_state_to_go_seg.y  << endl;
            robot_state r_state = r_state;  // get current state  
            go_point_xy_inter(relative,locks,stop_control, r_state, r_state_to_go_seg,drone,break_at_CL,break_info,false );

          }
          else  // absolut positions
          {              
            r_state_to_go_seg.x = r_state_0.x + Dx;
            r_state_to_go_seg.y = r_state_0.y + Dy;
            Dx = Dx + dx;
            Dy = Dy + dy;             
            cout << "-> I-point (a): " << r_state_to_go_seg.x << " " << -r_state_to_go_seg.y  << endl;
            robot_state r_state = r_state;  // get current state 
            go_point_xy_inter(relative,locks,stop_control, r_state, r_state_to_go_seg,drone,break_at_CL,break_info,false );

          }

       }
       // reach final position
       cout << "-> F-point (a): " << r_state_to_go.x << " " << -r_state_to_go.y  << endl;
       robot_state r_state = r_state;  // get current state 
       go_point_xy_inter(relative,locks,stop_control, r_state_0, r_state_to_go,drone,break_at_CL,break_info,true  );

   }
   else
   { 
        // for small displasments simply go to the point 
        // final point
        go_point_xy_inter(relative,locks,stop_control, r_state_0, r_state_to_go,drone,break_at_CL,break_info,true  );
   }

    
    
 

  
  return true;

}    


/*

//------------------------------------------------------------------------------------------
bool CONTROL::go_point2(bool relative,LOCKS &locks,bool &stop_control, robot_state r_state_0, robot_state r_state_to_go,vpRobotBebop2 &drone , bool break_at_CL, std::string &break_info)
{   
    // Use PI control


    // If relative == true, then:
    // the movement is executed relative to its current position.   
    bool arrive = false;
    bool init_pi = false;
    
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

        double norm_xy_e = sqrt(e(0)*e(0) + e(1)*e(1));
        double norm_z_e = abs(e(2));
        double norm_yaw_e = abs(e(3));         

         if(norm_xy_e < PAR.control.Max_error_xy_reach_p && norm_yaw_e < PAR.control.Max_error_yaw_reach_p && norm_z_e < PAR.control.Max_error_z_reach_p )
         {   
             drone.stopMoving(); 
             // if error is less than some threshold break
             cout << "-> point reached" << endl;
             return true;
             //break;
         }

        // set gains        
        gain Kp;
        Kp.x = 35;
        Kp.y = 35;
        Kp.z = 35;
        Kp.yaw = 100;

        gain Ki;
        Ki.x = .3;
        Ki.y = .3;
        Ki.z = 5;
        Ki.yaw = 0;        
        
        arma::vec::fixed<4> v_n;
        if(init_pi == false)
        {
            v_n = PI_control(xd, x, true, Kp, Ki ,.025);
            init_pi = true;
        }
        else
        {
           v_n = PI_control(xd, x, false, Kp, Ki ,.025); 
        }
              
        // transform velocities to bebop coodinate frame
        double a = x(3) + (3.1416/2) ;  // current yaw   
        arma::mat::fixed<2,2> Rb2n =  {{cos(a), -sin(a)},
                                           { sin(a), cos(a)}};
        arma::vec::fixed<2> v_xy_N;
        v_xy_N(0) =  v_n(0);
        v_xy_N(1) =  v_n(1);                              
           
       arma::vec::fixed<2> v_xy_R = Rb2n.t()*v_xy_N;

        arma::vec::fixed<4> v_b;
        v_b(0) = -v_xy_R(0);
        v_b(1) = -v_xy_R(1);        
        v_b(2) = -v_n(2);
        v_b(3) = v_n(3);        

        drone.setPitch(v_b(0)); // + Forward
        drone.setRoll(v_b(1));  //+ Right
        drone.setVerticalSpeed(v_b(2));  // + Up
        drone.setYawSpeed(v_b(3)); //  + Roll right

       // cout << "e: " << e(0) << " " << e(1) << " " << e(2) << " " << e(3)  << endl;        
      //  cout << "ve_n: " << v_n(0) << " " << v_n(1) << " " << v_n(2) << " " << v_n(3)  << endl;
      //  cout << "ve_r: " << v_b(0)  << " " << v_b(1)  << " " << v_b(2)  << " " << v_b(3)   << endl;
       
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
        usleep(25000);  // wait 25 milliseconds 
    }    
    drone.stopMoving();
    break_info = "stop_control";
    return false; 
}

//--------------------------------------------------------------------

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

          double norm_xy_e = sqrt(e(0)*e(0) + e(1)*e(1));
        double norm_z_e = abs(e(2));
        double norm_yaw_e = abs(e(3)); 

        //cout << e << endl;

         if(norm_xy_e < PAR.control.Max_error_xy_reach_p && norm_yaw_e < PAR.control.Max_error_yaw_reach_p && norm_z_e < PAR.control.Max_error_z_reach_p )
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
       
        // transform velocities to bebop coodinate frame
        double a = x(3) + (3.1416/2) ;  // current yaw   
        arma::mat::fixed<2,2> Rb2n =  {{cos(a), -sin(a)},
                                           { sin(a), cos(a)}};
        arma::vec::fixed<2> v_xy_N;
        v_xy_N(0) =  vx;
        v_xy_N(1) =  vy;                              
           
       arma::vec::fixed<2> v_xy_R = Rb2n.t()*v_xy_N;
        
        ve[0] = v_xy_R(0);
        ve[1] = v_xy_R(1);        
        ve[2] = -vz;
        ve[3] = -vyaw;

        // check for max velocities allowed
        if(ve[0] > PAR.control.MaxVel_xy ) ve[0] = PAR.control.MaxVel_xy;
        if(ve[0] < -PAR.control.MaxVel_xy ) ve[0] = -PAR.control.MaxVel_xy;
        if(ve[1] > PAR.control.MaxVel_xy ) ve[1] = PAR.control.MaxVel_xy;
        if(ve[1] < -PAR.control.MaxVel_xy ) ve[1] = -PAR.control.MaxVel_xy;
        if(ve[2] > PAR.control.MaxVel_z ) ve[2] = PAR.control.MaxVel_z;
        if(ve[2] < -PAR.control.MaxVel_z ) ve[2] = -PAR.control.MaxVel_z;
        if(ve[3] > PAR.control.MaxVel_yaw ) ve[3] = PAR.control.MaxVel_yaw;
        if(ve[3] < -PAR.control.MaxVel_yaw ) ve[3] = -PAR.control.MaxVel_yaw;
        
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


//-------------------------------------------------------------------------
arma::vec::fixed<4> CONTROL::PI_control(arma::vec::fixed<4> xd, arma::vec::fixed<4> x, bool flag, gain &Kp, gain &Ki,double Ts )
{
    static  arma::vec::fixed<4> integrator_e;
    static  arma::vec::fixed<4> error_d1;

    if(flag == true)
    {
        integrator_e(0) = 0;
        integrator_e(1) = 0;
        integrator_e(2) = 0;
        integrator_e(3) = 0;
        error_d1(0) = 0;
        error_d1(1) = 0;
        error_d1(2) = 0;
        error_d1(3) = 0;
    }
    arma::vec::fixed<4> error = xd - x; // error
    AngleWrap(error(3));

    integrator_e = integrator_e + (Ts/2)*(error + error_d1);
    error_d1 = error;

    arma::vec::fixed<4> u_unsat;
    u_unsat(0) = Kp.x*error(0) + Ki.x*integrator_e(0);
    u_unsat(1) = Kp.y*error(1) + Ki.y*integrator_e(1);
    u_unsat(2) = Kp.z*error(2) + Ki.z*integrator_e(2);
    u_unsat(3) = Kp.yaw*error(3) + Ki.yaw*integrator_e(3);

    arma::vec::fixed<4> u;
    u(0) = sat(u_unsat(0),PAR.control.MaxVel_xy2);
    u(1) = sat(u_unsat(1),PAR.control.MaxVel_xy2);
    u(2) = sat(u_unsat(2),PAR.control.MaxVel_z2);
    u(3) = sat(u_unsat(3),PAR.control.MaxVel_yaw2);

    // integrator anti-windup

    integrator_e(0) = integrator_e(0) + (Ts/Ki.x)*(u(0) - u_unsat(0));
    integrator_e(1) = integrator_e(1) + (Ts/Ki.y)*(u(1) - u_unsat(1));
    integrator_e(2) = integrator_e(2) + (Ts/Ki.z)*(u(2) - u_unsat(2));
    integrator_e(3) = integrator_e(3) + (Ts/Ki.yaw)*(u(3) - u_unsat(3));

   return u;

}

*/
