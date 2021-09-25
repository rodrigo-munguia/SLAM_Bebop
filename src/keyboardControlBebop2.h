#include "Bebop2.h"
#include "ekf/ekf.h"
#include "loop/loop.h"
#include "locks.h"
#include "parameters.h"


 void StartDataCapture(vpRobotBebop2 &drone);
 void StopDataCapture(vpRobotBebop2 &drone);
 void SingleFrameCapture(vpRobotBebop2 &drone);
 void StartSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks);
 void StopSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks);
 void printcommands();


//--------------------------------------------------
 bool handleKeyboardInput(vpRobotBebop2 &drone, int key, EKF &ekf, LOCKS &locks,parameters &par)
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
       drone.land();
       break;
 
     case 'i':
       // Up
       drone.setVerticalSpeed(10);
       break;
 
     case 'k':
       // Down
       drone.setVerticalSpeed(-10);
       break;
 
     case 'l':
       // Right
       drone.setYawSpeed(10);
       break;
 
     case 'j':
       // Left
       drone.setYawSpeed(-10);
       break;
 
     case 'r':
       // Forward
       drone.setPitch(10);
       break;
 
     case 'f':
       // Backward
       drone.setPitch(-10);
       break;
 
     case 'd':
       // Roll left
       drone.setRoll(-10);
       break;
 
     case 'g':
       // Roll right
       drone.setRoll(10);
       break;

     case '1':
       // Start data capture
       StartDataCapture(drone);
       break;
     
     case '2':
      // Stop data capture
       StopDataCapture(drone);
       break;

     case '3':
       // Single frame capture
       SingleFrameCapture(drone);
       break;

     case '7':
       // Downward camera
       drone.setCameraOrientation(-90,0,false);
       std::cout << "Downward camera " << std::endl;
       break;
     
     case '6':
       // forward camera
       drone.setCameraOrientation(0,0,false);
      std::cout << "Forward camera camera " << std::endl;
       break;

     case 'b':
       std::cout << "Battery level: " << drone.getBatteryLevel() << " percent"<< std::endl;
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
       StartSLAM(ekf,drone,par,locks);
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

//------------------------------------------------------
void StopSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks)
{
  std::cout << "Stop SLAM.." << std::endl;

  locks.ekf_run_mtx.lock();    
    ekf.run = false;
  locks.ekf_run_mtx.unlock();

}
void StartSLAM(EKF &ekf,vpRobotBebop2 &drone,parameters &par, LOCKS &locks)
{
  std::cout << "Initializing SLAM.." << std::endl;

  double bebop_cam_tilt_deg = drone.getCurrentCameraTilt(); 
  
  std::cout << "Camera tilt: " << bebop_cam_tilt_deg << std::endl;
  double ekf_camera_tilt_rad = (bebop_cam_tilt_deg + 90)*(3.1416/180);

  par.init.roll_init = ekf_camera_tilt_rad;


  locks.ekf_run_mtx.lock();    
    ekf.run = false;
  locks.ekf_run_mtx.unlock();
     sleep(1); // give time to finish the last ekf loop  

    ekf.state_init(); // initialize system state
  
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
                    "|   1-> start capture  2-> stop capture  3-> single frame capture\n"
                    "|   5-> Get GPS location 6-> Forward camera  7-> Downward camera  'b'-> Battery level 'c'->  Commands menu\n "
                    "|   8-> Stop SLAM  9-> Start SLAM\n "
                 << std::endl;
   
 }   

