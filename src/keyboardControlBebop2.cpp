/****************************************************************************
  *  Rodrigo M.  2021,  
  *
  *
  *****************************************************************************/
 #include "getData.h"

 //#include <visp3/core/vpConfig.h>
 
 //#include <visp3/core/vpTime.h>
 #include <visp3/gui/vpDisplayX.h>
 #include <visp3/io/vpKeyboard.h>
 
 #include "Bebop2.h"
 #include "Fullnavdata.h"
 #include "keyboardControlBebop2.h"
 #include "map/map.h"
 #include "ekf/ekf.h"
 #include "loop/loop.h"
 #include "locks.h"
 #include "parameters.h"
 #include "plot.h"

 #include <iostream>
  
 

/*
 void StartDataCapture(vpRobotBebop2 &drone);
 void StopDataCapture(vpRobotBebop2 &drone);
 void SingleFrameCapture(vpRobotBebop2 &drone);
 void printcommands();
 bool handleKeyboardInput(vpRobotBebop2 &drone, int key);
*/

void ekf_slam(EKF &ekf,GMAP &gmap,LOOP &cloop,parameters &par,LOCKS &locks,bool &running);

void plot_f(PLOT &plot,EKF &ekf,GMAP &gmap,parameters &par,bool &running,LOCKS &locks);

void mapping(GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks,bool &running);

void loop(LOOP &cloop, GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks,bool &running);

 
 int main(int argc, char **argv)
 {
   try {
 
     std::string ip_address = "192.168.42.1";
 
     int stream_res = 0; // Default 480p resolution
 
     bool verbose = false;
 
     for (int i = 1; i < argc; i++) {
       if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
         ip_address = std::string(argv[i + 1]);
         i++;
       } else if (std::string(argv[i]) == "--hd_stream") {
         stream_res = 1;
       } else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
         verbose = true;
       } else if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
         std::cout << "\nUsage:\n"
                   << "  " << argv[0] << "[--ip <drone ip>] [--hd_stream] [--verbose] [-v] [--help] [-h]\n"
                   << std::endl
                   << "Description:\n"
                   << "  --ip <drone ip>\n"
                   << "      Ip address of the drone to which you want to connect (default : 192.168.42.1).\n\n"
                   << "  --hd_stream\n"
                   << "      Enables HD 720p streaming instead of default 480p.\n"
                   << "  --verbose, -v\n"
                   << "      Enables verbose (drone information messages and velocity commands\n"
                   << "      are then displayed).\n\n"
                   << "  --help, -h\n"
                   << "      Print help message.\n"
                   << std::endl;
         return 0;
       } else {
         std::cout << "Error : unknown parameter " << argv[i] << std::endl
                   << "See " << argv[0] << " --help" << std::endl;
         return 0;
       }
     }
 
     std::cout << "Connecting to Drone...\n"
               << std::endl;

    //PLOT p;
    //p.init();
    

 
     vpRobotBebop2 drone(verbose, true,
                         ip_address); // Create the drone with low verbose level, settings reset and corresponding IP
     
     

     //---------------------------
     
     if (drone.isRunning()) {

        //---------------------------
        //-- Initialize
        parameters par;
        LOCKS locks;
        par = get_parameters(); // get parameters structure
        EKF ekf(par); // create EKF-SLAM object  
        GMAP gmap(par); // create Global MAP object
        LOOP cloop(par); // create Closing loop opbject
        PLOT plot(par);

        GetData data;   // set callbacks
        drone._navdata->SetReceivedRangeCallback(&data.ReceiveRangeCallBack); 
        drone.SetReceivedAltitudeCallback(&data.ReceiveAltitudeCallBack);
        drone.SetReceivedFrameCallback(&data.ReceiveFrameCallBack); // Set Callback for frame     
        drone.SetReceivedSpeedCallback(&data.ReceiveSpeedCallback);
        drone.SetReceivedAttitudeCallback(&data.ReceiveAttitudeCallback);
        
        
       
      int k = 0;
        bool running = true;
        ekf.run = false;

        // init EKF-SLAM thread
        thread th1(ekf_slam,std::ref(ekf),std::ref(gmap),std::ref(cloop),std::ref(par),std::ref(locks),std::ref(running));
        
        // init Mapping thread
        thread th2(mapping,std::ref(gmap),std::ref(par),std::ref(ekf),std::ref(locks),std::ref(running));
        
        // init Closing-loop thread
        thread th3(loop,std::ref(cloop),std::ref(gmap),std::ref(par),std::ref(ekf),std::ref(locks),std::ref(running));

        // init Plot thread     
        thread th4(plot_f,std::ref(plot),std::ref(ekf),std::ref(gmap),std::ref(par),std::ref(running),std::ref(locks));
       

       
 
       std::cout << "\nConfiguring drone settings ...\n" << std::endl;
 
       drone.setMaxTilt(10); // Setting the max roll and pitch values, the drone speed will depend on it
 
       drone.doFlatTrim(); // Flat trim calibration

       // Downward camera
       drone.setCameraOrientation(-90,0,false);
 
      #ifdef VISP_HAVE_FFMPEG
          drone.setVideoResolution(stream_res); // Setting desired stream video resolution
          drone.setStreamingMode(0); // Set streaming mode 0 : lowest latency
          std::cout << "\nWaiting for streaming to start ...\n" << std::endl;
          drone.startStreaming();
          drone.setMaxTilt(80);
    
          // Prepare image for display
        //  vpImage<vpRGBa> I(1, 1, 0);
        //  drone.getRGBaImage(I);
        //  vpDisplayX display(I, 100, 100, "DRONE VIEW");
        //  vpDisplay::display(I);
        //  vpDisplay::flush(I);
      #endif

       drone.useFullNavdata();
      sleep(1);
      drone.isUsingFullNavdata();
 
       vpKeyboard keyboard;
       
       printcommands();

               
      // Main control loop ----------------------------------------------------
      while (running && drone.isRunning() && drone.isStreaming()) 
      {
 
         k = '0'; // If no key is hit, we send a non-assigned key
         if (keyboard.kbhit()) 
         {
           k = keyboard.getchar();
         }
         running = handleKeyboardInput(drone, k,ekf,locks,par);
         

          #ifdef VISP_HAVE_FFMPEG
         //     drone.getRGBaImage(I);
         //     vpDisplay::display(I);
         //     vpDisplay::displayText(I, 10, 10, "Press q to quit", vpColor::red);
         //     vpDisplay::flush(I);
          #endif
       }
       ekf.run = false;
       //---------------------------------------------------------------------
       std::cout << "\nQuitting ...\n" << std::endl;
       // Wait for thread t1 to finish     
      th1.join();
      th2.join();
      th3.join();
      th4.join();
 
     } else {
       std::cout << "ERROR : failed to setup drone control." << std::endl;
       return EXIT_FAILURE;
     }
   } catch (const vpException &e) {
     std::cout << "\nCaught an exception: " << e << std::endl;
     return EXIT_FAILURE;
   }
 }
//-------------------------------------------------------------------------
// Function thread for loop closing
//-------------------------------------------------------------------------
void loop(LOOP &cloop, GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks,bool &running)
{
  
  cout << "Loop thread running... " << endl;  
  bool wait = true; 
  while(wait == true)
  { 
    sleep(1);   
  
      while(ekf.run == true &&  ekf.Initialized == true)
        {
          // gmap.update(locks);
          if(cloop.newFrame == true && par.sys.closing_loop_active == true)
          {
            cloop.update(gmap,locks);           
          }     
        }
    if(running == false)
    {
      wait = false;
    }
  }      

}

//-------------------------------------------------------------------------
// Function thread for Mapping using optimization techniques
//-------------------------------------------------------------------------
void mapping(GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks,bool &running)
{
  cout << "Mapping thread running... " << endl;  
  bool wait = true; 
  while(wait == true)
  { 
    sleep(1);  
   
    while(ekf.run == true && ekf.Initialized == true)
    {
        if(gmap.closing_loop_active == false )  
        { 
            gmap.update(locks);
        }

    }    
    if(running == false)
    {
      wait = false;
    }
  }

  
}  

//-------------------------------------------------------------------------
// Function thread for EKF-SLAM
//-------------------------------------------------------------------------
void ekf_slam(EKF &ekf,GMAP &gmap,LOOP &cloop,parameters &par,LOCKS &locks,bool &running)
{

    cout << "EKF-SLAM thread running... " << endl;   

    DATA dat;
    
    
    auto last_time = std::chrono::high_resolution_clock::now();
    bool init = false;
    double delta_t;
   
    
  while(running == true) // program is running
  { 
    //sleep(1);
    while(ekf.run == true)  // ekf is running
    {            
            
      auto t_c = std::chrono::high_resolution_clock::now();
            
      // compute delta time-----
      if (ekf.Initialized == false)
        { 
          
          delta_t = par.ekf.delta_t;
          last_time = t_c;
          dat = GetData::getDataB();  // Get Measurements 
          init = ekf.System_init(dat); // initialize system state
          //init = true;
        }
      else // Until system is initialized
        { 
          

          auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t_c - last_time);
          delta_t = elapsed.count() * 1e-9;
          last_time = t_c;
           
          ekf.prediction(delta_t); // EKF prediction              

          dat = GetData::getDataB();  // Get Measurements           
          
          if(dat.data_type == "frame")
          {     
                // Start measuring time
              auto begin = std::chrono::high_resolution_clock::now();             

              ekf.visual_update(&dat.frame,gmap,cloop,locks); // EKF visual update 

              ekf.store_data_for_plot(locks,&dat.frame);
              
              auto end = std::chrono::high_resolution_clock::now();
              auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

            //std::printf("Time per frame: %.5f seconds.  ", elapsed.count() * 1e-9);
              
              //cout << "n feats: " << ekf.FeatsDATA.size() << "  n anchors: " << ekf.AnchorsDATA.size() ;
              //cout << "  x: " << ekf.x(7) << "  y:" << ekf.x(8) << "  z:" << ekf.x(9) << endl;       
            
            //cout << "frame" <<  " range: " << dat.frame.range << "  range type: " << dat.frame.range_type << endl;

          }
          if(dat.data_type == "alt")
          {
              ekf.altitude_update_alt(dat.alt);  
            //cout << "altitude: " << dat.alt.altitude << endl;
          }
          if(dat.data_type == "speed")
          {            
            ekf.speed_update(dat.speed);
            // cout << "speedX: " << dat.speed.speedX << " speedY: " << dat.speed.speedY << " speedZ:" << dat.speed.speedZ << endl;
            // cout << "  x: " << ekf.x(7) << "  y:" << ekf.x(8) << "  z:" << ekf.x(9) << endl;

          }
          if(dat.data_type == "attitude")
          {          
            ekf.attitude_update(dat.att);
            //cout << "roll: " << dat.att.roll << " pitch: " << dat.att.pitch << " yaw:" << dat.att.yaw << endl;
            //cout << ekf.yaw_at_home << endl;
          }

        } // else (init ==true)


       
    }

  }  

}

//--------------------------------------------------------------------
void plot_f(PLOT &plot, EKF &ekf,GMAP &gmap,parameters &par,bool &running,LOCKS &locks)
{

  cout << "Plot thread running... " << endl;  
  bool wait = true; 
  while(wait == true)
  { 
    sleep(1);  
    if(ekf.Initialized== true)
    {
       wait = false;
    }
    if(running == false)
    {
      wait = false;
    }
  }

  
  

  if(ekf.Initialized== true)
   {
   plot.init(ekf,gmap,locks,running);
   }

  // sleep(1);


 // while(running == true)
 // { 
    // while(!plot.viewer->wasStopped())
       // plot.Update(ekf,gmap,par,running);
    
 // }


}