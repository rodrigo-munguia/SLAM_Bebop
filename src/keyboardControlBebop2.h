#include "Bebop2.h"
#include "ekf/ekf.h"
#include "loop/loop.h"
#include "locks.h"
#include "parameters.h"
#include "control/control.h"


 void StartDataCapture(vpRobotBebop2 &drone);
 void StopDataCapture(vpRobotBebop2 &drone);
 void SingleFrameCapture(vpRobotBebop2 &drone);
 void StartSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks,GMAP &gmap,LOOP &cloop);
 void StopSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks);
 void printcommands();
 void StartControlPlan(EKF &ekf,CONTROL &control);


//--------------------------------------------------
 bool handleKeyboardInput(vpRobotBebop2 &drone, int key, EKF &ekf, LOCKS &locks,parameters &par,GMAP &gmap,LOOP &cloop,CONTROL &control,bool &stop_control)
 {
   bool running = true;
   if (drone.isRunning()) {
     switch (key) {
     case 'q':
       // Quit
       drone.land();
       running = false;
       break;
 
     case 'w':
       // Emergency
       drone.cutMotors();
       running = false;
       break;
 
     case 't':
       // Takeoff
       drone.takeOff(false);
       break;
 
     case ' ':
       // Landing
       stop_control = true; 
       drone.land();
       break;
 
     case 'i':
       // Up
       drone.setVerticalSpeed(20);
       break;
 
     case 'k':
       // Down
       drone.setVerticalSpeed(-20);
       break;
 
     case 'l':
       // Right
       drone.setYawSpeed(50);
       break;
 
     case 'j':
       // Left
       drone.setYawSpeed(-50);
       break;
 
     case 'r':
       // Forward
       drone.setPitch(50);
       break;
 
     case 'f':
       // Backward
       drone.setPitch(-50);
       break;
 
     case 'd':
       // Roll left
       drone.setRoll(-50);
       break;
 
     case 'g':
       // Roll right
       drone.setRoll(50);
       break;

     case '1':
       // Start control plan
       StartControlPlan(ekf,control);
      
       break;
     
     case '2':
      // Stop control plan!!
       stop_control = true;        
       break;

     case '3':
       // Single frame capture
       SingleFrameCapture(drone);
       break;

     case '7':
       // Downward camera
       drone.setCameraOrientation(-90,0,false);
       std::cout << "-> Downward camera " << std::endl;
       break;
     
     case '6':
       // forward camera
       drone.setCameraOrientation(0,0,false);
      std::cout << "-> Forward camera camera " << std::endl;
       break;

     case 'b':
       std::cout << "-> Battery level: " << drone.getBatteryLevel() << " percent"<< std::endl;
       break;

     case 'c':
       printcommands(); // print commands options
       break;

     case '5':  // Get drone current location
       std::cout << std::setprecision( 6 ) << "Lat: " << (float)drone.getLatitude() << "  Lon: " << (float)drone.getLongitude() << "  Alt: " << (float)drone.getAltitude() << std::endl;
        break;
     
     case '8': // Stop SLAM
        StopSLAM(ekf,drone,par,locks);
        break;

     case '9': // Start SLAM
       StartSLAM(ekf,drone,par,locks,gmap,cloop);
       break;                     
 
     default:
       // No inputs -> drone stops moving
       drone.stopMoving();
       break;
     }
     vpTime::wait(25); // We wait 25ms to give the drone the time to process the command
   } else {
     running = false;
   }
   return running;
 }

//----------------------------------------------------------
 void StartControlPlan(EKF &ekf,CONTROL &control)
 {
  
  if(ekf.Initialized == true)
  {
    control.execute_control_plan = true;

  }
  else
  {
    cout << "-> SLAM is not running!" << endl;
  }


 }


//------------------------------------------------------
void StopSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks)
{
  std::cout << "-> Stop SLAM.." << std::endl;

  locks.ekf_run_mtx.lock();    
    ekf.run = false;
    ekf.Initialized = false; // Every time SLAM is stoped, the system must be reinitialized    
  locks.ekf_run_mtx.unlock();

}
void StartSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks,GMAP &gmap,LOOP &cloop)
{
  

  double bebop_cam_tilt_deg = drone.getCurrentCameraTilt();                               
  
  //bebop_cam_tilt_deg = -45;

  std::cout << "Camera tilt: " << bebop_cam_tilt_deg << std::endl;
  double ekf_camera_tilt_rad = (bebop_cam_tilt_deg + 90)*(3.1416/180);

  ekf.Init_cam_position.axis_x = ekf_camera_tilt_rad;
  //ekf.Init_cam_position.axis_x = 45*(3.1416/180);       
  
  //ekf.Init_cam_position.axis_y = par.init.roll_init;
  ekf.Init_cam_position.axis_y = 0; 
  //ekf.Init_cam_position.axis_z = par.init.yaw_init;
  ekf.Init_cam_position.axis_z = 0;
  
  ekf.Init_cam_position.x = par.init.x_init;
  ekf.Init_cam_position.y = par.init.y_init;
  ekf.Init_cam_position.z = par.init.z_init;


  std::cout << "-> Initializing EKF-SLAM.." << std::endl;
  
  locks.ekf_run_mtx.lock();    
    ekf.run = false;
    ekf.Initialized = false; // Every time SLAM is started, the system must be reinitialized
  locks.ekf_run_mtx.unlock();
    
    gmap.reset();
    cloop.reset();
    
    sleep(1); // give time to finish the last ekf loop  

    //ekf.state_init(); // initialize system state
  
    ekf.run = true;
  

}
// Rodrigo M
void StartDataCapture(vpRobotBebop2 &drone)
 {
  if (drone.DataCapture == false)
  {
      std::cout << "Data capture started" << std::endl;
      time_t t = time(NULL);
      tm* timePtr = localtime(&t);
      
      
      char dir[100];
            sprintf(dir, "data/%d-%d-%d-%d-%d/", timePtr->tm_year+1900,timePtr->tm_mon,timePtr->tm_mday,timePtr->tm_hour,timePtr->tm_min );
            mkdir(dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

            
      strncpy(drone.StoreDir, dir, 100);
      
      drone.DataCapture = true;
      drone.StartFramesCapture();  // start frames capture
      drone.StartNavDataCapture(); // start Navigation data capture (from Fullnavdata: See Bebop2cpp)

   
  } 
 }

 //----------------------------
 




 //-------------
 //-------------------------
 void StopDataCapture(vpRobotBebop2 &drone)
 { 
  
  if (drone.DataCapture == true)
  {
    std::cout << "Data capture Stoped" << std::endl;
    drone.DataCapture = false;
    drone.StopFramesCapture(); // stop frames capture
    drone.StopNavDataCapture(); // stop navdata capture

  }  

 }
 //-------------------------
 void SingleFrameCapture(vpRobotBebop2 &drone)
 {
   std::cout << "Frame captured" << std::endl;
   drone.CaptureSingleFrame();  // capture single frame
 }
///---------------------------------
 void printcommands()
 {
    std::cout << "\n| Control the drone with the keyboard :\n"
                    "|   't' to takeoff / spacebar to land / 'w' for emergency stop\n"
                    "|   ('r','f','d','g') and ('i','k','j','l') to move\n"
                    "|   'q' to quit.\n"
                    "|   1-> Start control plan  2-> Stop control plan 3-> single frame capture\n"
                    "|   5-> Get GPS location 6-> Forward camera  7-> Downward camera  'b'-> Battery level 'c'->  Commands menu\n "
                    "|   8-> Stop SLAM  9-> Start SLAM\n "
                 << std::endl;
   
 }   

