#ifndef GET_DATA_H
#define GET_DATA_H


#include "opencv2/opencv.hpp"
#include <string>
#include <mutex>


using namespace std;

// Speed  data
//speedX (float): Speed on the x axis (when drone moves forward, speed is > 0) (in m/s)
//speedY (float): Speed on the y axis (when drone moves to right, speed is > 0) (in m/s)
//speedZ (float): Speed on the z axis (when drone moves down, speed is > 0) (in m/s)

struct ATT
{
    long int time;
    float roll; //  
    float pitch;
    float yaw;
};


struct SPD
{
    long int time;
    float speedX; //  (m/s)
    float speedY;
    float speedZ;
};

// gps data
struct GPS
{
   long int time;
   double lat;
   double lon;
   double alt;
   int sat;     
};

// barometer (raw) data
struct BAR 
{
    long int time;
    double pressure;
    double temp;
};
// Altitude  data
struct ALT
{
    long int time;
    double altitude; //  (meters)
};


// Range (sonar) data
struct RANGE
{
    long int time;
    double range;
    double volt;
};

// Visual data
struct FRAME
{
    long int time;
    string image_file;
    cv::Mat image;
    double range;
    string range_type;
};


// struct for storing data from sensors
struct DATA
{   
    string data_type; 
    //GPS gps;
    //BAR bar;
    RANGE range;
    FRAME frame;
    ALT alt;
    SPD speed;
    ATT att;
};

/*
struct DATA
{  
  string data_type; 
  
  cv::Mat frame;
  double altitude;
  double range;


};  
*/



class GetData
{


 

public:
  
  
  static std::mutex mutex_dat;
  
  static vector<DATA> DataBuffer; 
  
  static void ReceiveRangeCallBack(double range);
  static void GetData::ReceiveAltitudeCallBack(double altitude);
  static void GetData::ReceiveFrameCallBack(cv::Mat& frame);
  static void GetData::ReceiveSpeedCallback(float speedX,float speedY,float speedZ);
  static void GetData::ReceiveAttitudeCallback(float roll,float pitch,float yaw);
  static DATA getDataB();

private:

  static void ManageBuffer(DATA &dat);

  

};  
 


#endif 