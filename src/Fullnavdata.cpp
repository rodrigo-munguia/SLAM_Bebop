/** TODO MIT **/

#include "Fullnavdata.h"

#include <vector>
#include <utility>
#include <iostream>
#include <chrono>
#include <memory>
#include <array>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;

Fullnavdata::Fullnavdata()
    :_spinlock(ATOMIC_FLAG_INIT)
{
}

Fullnavdata::~Fullnavdata() {
    if(_updater != NULL){
        _ioService.stop();
        delete _updater;
    }
}


void Fullnavdata::init(std::string ip, int senderPort)
{
    std::cout << "INIT" << std::endl;
    _navdata_socket.reset(new udp::socket(_ioService, udp::v4()));
    _navdata_socket->bind(udp::endpoint(udp::v4(), senderPort));
    std::cout << "bind ok" << std::endl;


    udp::resolver resolver(_ioService);
    udp::resolver::query query(ip, std::to_string(FULL_NAVDATA_DEFAULT_PORT));

    std::cout << "resolving " << query.host_name() << ":" << query.service_name() << std::endl;

    endpoint = *resolver.resolve(query);
    //lol
    std::cout << "sending to " << endpoint.address() << ":" << endpoint.protocol().protocol() << std::endl;

    size_t sent = _navdata_socket->send_to(boost::asio::buffer("hello"), endpoint);

    std::cout << sent << " bytes sent" << std::endl;

    // Start receiving beautiful, full, informative (and not dumbed-down by Parrot) navdata ;)
    std::cout << "ASYNC WAITING" << std::endl;
   
   
    _navdata_socket->async_receive_from(
            boost::asio::buffer(_navdata_buf),
            _navdata_sender_endpoint,
            boost::bind(&Fullnavdata::navdataPacketReceived,
                        this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred
            )
    );
    
    /*
    while(1) {
        size_t read = _navdata_socket->receive_from(
                boost::asio::buffer(_navdata_buf, FULL_NAVDATA_MAX_SIZE),
                _navdata_sender_endpoint
        );
        navdataPacketReceived(boost::system::error_code(), read);
    }
     */
    std::cout << _navdata_sender_endpoint << std::endl;

    //_g->showonscreen();
}


void Fullnavdata::navdataPacketReceived(const boost::system::error_code &error, size_t bytes_transferred)
{
     struct timespec tpe;  // create struct timespec
     clock_gettime(CLOCK_REALTIME, &tpe); // get the current time
    
    
    while(_spinlock.test_and_set(std::memory_order_acquire));

    if(bytes_transferred < 1000)
    {
        return;
    }

    _received_time_computer = std::chrono::duration_cast<std::chrono::microseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();

    _gotNavdata = true;

     
    static double last_sensor_ultrasound_height_m = 0;

    //warning: data type size must not be over 8 bytes
    double acc_filt_x_m_s2;
    double acc_filt_y_m_s2;
    double acc_filt_z_m_s2;
    
    double sensor_acc_raw_x_m_s2;
    double sensor_acc_raw_y_m_s2;
    double sensor_acc_raw_z_m_s2;

    double sensor_gyro_raw_x_rad_s;
    double sensor_gyro_raw_y_rad_s;
    double sensor_gyro_raw_z_rad_s;

    double sensor_mag_raw_x_mG;
    double sensor_mag_raw_y_mG;
    double sensor_mag_raw_z_mG;

    double gyro_filt_x_rad_s;
    double gyro_filt_y_rad_s;
    double gyro_filt_z_rad_s;

    double sensor_ultrasound_height_m;

    double pressureAltitude_filt_comp_m;

    double speed_body_x_m_s;
    double speed_body_y_m_s;
    double speed_body_z_m_s;

    double test_x;
    double test_y;
    double test_z;

    double sent_time;


    int one = 16; //speed_ned_ref_x


    std::vector<std::pair<double *, int>> data_table = { // Firmware 4.0.6
            {&acc_filt_x_m_s2, 16*8},         //ok  
            {&acc_filt_y_m_s2, 17*8},         //ok
            {&acc_filt_z_m_s2, 18*8},         //ok

            {&sensor_acc_raw_x_m_s2, 16*8},
            {&sensor_acc_raw_y_m_s2, 17*8},
            {&sensor_acc_raw_z_m_s2, 18*8},

            {&sensor_gyro_raw_x_rad_s, 1352},       
            {&sensor_gyro_raw_y_rad_s, 1360},       
            {&sensor_gyro_raw_z_rad_s, 1368},       

            {&sensor_mag_raw_x_mG, 128},
            {&sensor_mag_raw_y_mG, 1400},
            {&sensor_mag_raw_z_mG, 1408},

            {&gyro_filt_x_rad_s, 76*8},              //ok
            {&gyro_filt_y_rad_s, 77*8},              //ok
            {&gyro_filt_z_rad_s, 78*8},              //ok

            {&sensor_ultrasound_height_m, 214*8},    //ok
            //{&inputs.hal.HAL_ultrasound_SI.altitude, 147*8},    //ok

            {&pressureAltitude_filt_comp_m, 196*8},

            {&speed_body_x_m_s, 226*8},              //ok
            {&speed_body_y_m_s, 227*8},              //ok
            {&speed_body_z_m_s, 228*8},              //ok

            {&sent_time, 1608},

            {&test_x, one},
            {&test_y, one+FULL_NAVDATA_DATASIZE},
            {&test_z, one+(2*FULL_NAVDATA_DATASIZE)}

    };

    for(const std::pair<double *, int> &element : data_table)
    {
        memcpy(
                element.first,
                _navdata_buf.data() + element.second,
                FULL_NAVDATA_DATASIZE
        );
    }

    _sent_drone_uptime = (int64_t) sent_time;

    _accelerometer_raw = Eigen::Vector3f(
            (float)sensor_acc_raw_x_m_s2,
            (float)sensor_acc_raw_y_m_s2,
            (float)sensor_acc_raw_z_m_s2
    );

    _gyroscope_raw = Eigen::Vector3f(
            (float)sensor_gyro_raw_x_rad_s,
            (float)sensor_gyro_raw_y_rad_s,
            (float)sensor_gyro_raw_z_rad_s
    );
    _magnetometer_raw = Eigen::Vector3f(
            (float)sensor_mag_raw_x_mG,
            (float)sensor_mag_raw_y_mG,
            (float)sensor_mag_raw_z_mG
    );
    _gyroscope_filt = Eigen::Vector3f(
            (float)gyro_filt_x_rad_s,
            (float)gyro_filt_y_rad_s,
            (float)gyro_filt_z_rad_s
    );

    _test = Eigen::Vector3f(
            (float)test_x,
            (float)test_y,
            (float)test_z
    );

    _body_speed = Eigen::Vector3f(
            (float)speed_body_x_m_s,
            (float)speed_body_y_m_s,
            (float)speed_body_z_m_s
    );


    _height_ultrasonic = (float) sensor_ultrasound_height_m;

   
   
   if (sensor_ultrasound_height_m != last_sensor_ultrasound_height_m)
   {

        ReceivedRangeCallback(sensor_ultrasound_height_m);

   } 
    
    // Store Data capture
   if ((Datacapture == true)&&(sensor_ultrasound_height_m != last_sensor_ultrasound_height_m))
   {
     long us;
     us = tpe.tv_nsec/1000;
     long sec = tpe.tv_sec - (tpe.tv_sec/10000)*10000;
     unsigned long time_in_micros = (1000000 * sec) + us;
     
     fileRangeSensor << time_in_micros <<"," << std::setprecision(6) << sensor_ultrasound_height_m << "," << 0 <<"\n";
     
     last_sensor_ultrasound_height_m = sensor_ultrasound_height_m;
   }

   //std::cout << pressureAltitude_filt_comp_m << std::endl;
    

    // Listen for next packet
    _spinlock.clear(std::memory_order_release);
    _navdata_socket->async_receive_from(boost::asio::buffer(_navdata_buf), _navdata_sender_endpoint, boost::bind(&Fullnavdata::navdataPacketReceived, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}
//--------------------
void Fullnavdata::StartNavDataCapture()
{
   // Create log files for data capture       

    char dir_file_name1[100];
    strcpy(dir_file_name1,  StoreDir);  // 
    strcat(dir_file_name1, "RangeSensor.txt");
    fileRangeSensor.open (dir_file_name1);

    char dir_file_name2[100];
    strcpy(dir_file_name2,  StoreDir);  // 
    strcat(dir_file_name2, "PressureSensor.txt");
    filePressureSensor.open (dir_file_name2);

}
void Fullnavdata::StopNavDataCapture()
{
   fileRangeSensor.close();
   filePressureSensor.close();

}

//-----------------------
void Fullnavdata::runUpdateLoop() {
    _ioService.run();
}


void Fullnavdata::startReceive() {
    if(_updater == nullptr){
        _updater = new boost::thread(&Fullnavdata::runUpdateLoop, this);
    }
}
/// ****** LOCKER
void Fullnavdata::lock(){
    while(_spinlock.test_and_set(std::memory_order_acquire));
}

void Fullnavdata::release(){
    _spinlock.clear(std::memory_order_release);
}

/// ****** GETTERS

bool Fullnavdata::receivedData() const {
    return _gotNavdata;
}

Eigen::Vector3f Fullnavdata::get_accelerometer_raw() const {
    return _accelerometer_raw;
}

Eigen::Vector3f Fullnavdata::get_gyroscope_raw() const {
    return _gyroscope_raw;
}

float Fullnavdata::get_height_ultrasonic() const {
    return _height_ultrasonic;
}

Eigen::Vector3f Fullnavdata::get_body_speed() const {
    return _body_speed;
}

Eigen::Vector3f Fullnavdata::get_magnetometer_raw() const {
    return _magnetometer_raw;
}

Eigen::Vector3f Fullnavdata::get_gyroscope_filt() const {
    return _gyroscope_filt;
}

Eigen::Vector3f Fullnavdata::get_test() const {
    return _test;
}

int64_t Fullnavdata::get_sent_drone_uptime() const {
    return _sent_drone_uptime;
}

int64_t Fullnavdata::get_received_time_computer() const {
    return _received_time_computer;
}
