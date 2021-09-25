#include "getData.h"


vector<DATA> GetData::DataBuffer;
std::mutex GetData::mutex_dat;

static void GetData::ReceiveSpeedCallback(float speedX,float speedY,float speedZ)
{

  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  
  DATA dat;
  dat.speed.time = ms;
  dat.data_type = "speed";
  dat.speed.speedX = -speedX;
  dat.speed.speedY = -speedY;
  dat.speed.speedZ = -speedZ;

  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock(); 

}

static void GetData::ReceiveAttitudeCallback(float roll,float pitch,float yaw)
 {

  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;

  DATA dat;
  dat.att.time = ms;
  dat.data_type = "attitude";
  dat.att.roll = roll;
  dat.att.pitch = pitch;
  dat.att.yaw = yaw; 

   mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock();


 }  



static void GetData::ReceiveRangeCallBack(double range)
 {
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;
  
  DATA dat;
  dat.range.time = ms;
  dat.data_type = "range";
  dat.range.range = range;

  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock(); 
 //std::cout << "R range: " << range << std::endl;
  
 }
 
 static void GetData::ReceiveAltitudeCallBack(double altitude)
 {
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;

  DATA dat;
  dat.alt.time = ms;
  dat.data_type = "alt";  
  dat.alt.altitude = altitude;

  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock();  
  //std::cout << "altitude: " << altitude << std::endl;
  
 }

 

 static void GetData::ReceiveFrameCallBack(cv::Mat& frame)
 {
  
  struct timespec tpe;  // create struct timespec
  clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
  long ms;
  ms = tpe.tv_sec * 1000 + tpe.tv_nsec / 1e6;

  DATA dat;
  dat.frame.time = ms;
  dat.data_type = "frame";
  dat.frame.image = frame;
  dat.frame.range = -1; // by default
  
  
  
  
  mutex_dat.lock(); 
    ManageBuffer(dat);
  mutex_dat.unlock();  
  

  //std::cout << "frame received" << std::endl;
  
 }

 static void GetData::ManageBuffer(DATA &dat)
 {
   
   string data_type = dat.data_type;

 for (int i=0; i < DataBuffer.size(); i++)
 {
   if (DataBuffer[i].data_type == data_type)
   {
      DataBuffer.erase(DataBuffer.begin() + i);
      i--;
   }
 }

   DataBuffer.push_back(dat);
   
   /*
   for(int i=0; i < DataBuffer.size(); i++)
   {
     if(DataBuffer[i].data_type == "frame") 
          cout << "frame" << endl;
     if(DataBuffer[i].data_type == "alt") 
          cout << "altitude: " << DataBuffer[i].altitude  << endl;
     if(DataBuffer[i].data_type == "range") 
          cout << "range: " << DataBuffer[i].range  << endl;

   } 
   */
   

 }

 static DATA GetData::getDataB()
 {
    static double last_range;
    static long int last_range_time;
    static bool range_available = false;
    static bool range_available_alt = false ;
    static double last_alt = 0; 
    
  mutex_dat.lock();   
    
    DATA dat;

    for (int i= 0 ; i <DataBuffer.size() ; i++)
    {
      
      //--------Speed measurement
      if (DataBuffer[i].data_type == "speed")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);          
          break;
      }
      //-------- Attitude
      if (DataBuffer[i].data_type == "attitude")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);          
          break;
      }
      //------- altitude measurement
      if (DataBuffer[i].data_type == "alt")
      {   
          dat =  DataBuffer[i];        
          DataBuffer.erase(DataBuffer.begin() + i);
          last_alt = dat.alt.altitude;
          range_available_alt = true;
          break;
      }
      //-------- range measurement
      if (DataBuffer[i].data_type == "range")
      {   
          dat =  DataBuffer[i];
          DataBuffer.erase(DataBuffer.begin() + i);

          if (dat.range.range > .4 && dat.range.range < 6) // if sonar measurement is valid
          {
            last_range = dat.range.range;
            last_range_time = dat.range.time;
            range_available = true;           
            break;
          }
          else
          {
            dat.data_type = "null";
            break;
          }  
      }
      //-------- camera measurment
      if (DataBuffer[i].data_type == "frame")
      {   
          dat =  DataBuffer[i];                 
          DataBuffer.erase(DataBuffer.begin() + i);          

          if (range_available == true)
          {
            dat.frame.range = last_range; // if a range masurement has just "received" then associate it to the frame
            dat.frame.range_type = "sonar";
            range_available = false;  
          }  
          long int dt = dat.frame.time - last_range_time; //ms
          // if no range (from sensor) has been received fro a while, try to use alt measurements
          if ((range_available_alt == true) && (dt > 100) && (last_alt > 0) )
          {
            dat.frame.range = last_alt;
            dat.frame.range_type = "alt";
            range_available_alt = false;
          }
          break;
      }


    }  
  mutex_dat.unlock();

  return dat;
 }

