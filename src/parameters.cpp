#include "parameters.h"






parameters get_parameters()
{
    parameters PAR;
    
    PAR.init.DataSetType = 1;  //  2-> Bebop2 parameters    
    
    if (PAR.init.DataSetType == 1) BEBOPparameters(PAR);
    


    return PAR;
}

void BEBOPparameters(parameters &PAR)
{   
   
    
    //PAR.init.GPS_init_yaw = 98*(3.1416/180);  // For aligning GPS points with local coordinates 
    PAR.init.axis_x_init = 0;
    PAR.init.axis_y_init = 0;
    PAR.init.axis_z_init = 0*(3.1416/180);  //     
    PAR.init.x_init = 0;
    PAR.init.y_init = 0;
    PAR.init.z_init = 0;

    
    // general system parameters
    PAR.sys.closing_loop_active = true;

    PAR.sys.EKF_initialize_anchors = true;
    PAR.sys.EKF_use_anchors_from_GMAP = false;
    PAR.sys.GMAP_use_anchors_from_EKF = true; // (overwrite EKF_initialize_anchors )
    
    PAR.sys.GMAP_update_optimized_anchors = true;
    PAR.sys.GMAP_optimize_Keyframe_pos = false;
    PAR.sys.GMAP_update_optimized_Keyframe_pos = false; // (overwrite GMAP_optimize_Keyframe_pos )
    
    //--------------------------------------------------
    if( PAR.sys.GMAP_use_anchors_from_EKF == true) 
        PAR.sys.EKF_initialize_anchors = true;

    if(PAR.sys.GMAP_update_optimized_Keyframe_pos == true)
        PAR.sys.GMAP_optimize_Keyframe_pos = true;    

    // EKF parameters
    PAR.ekf.delta_t = 1/(double)24;
    PAR.ekf.sigma_a = 1; //2
    PAR.ekf.sigma_w = .001; //.001
    PAR.ekf.Tau_r = 1; // 1
    PAR.ekf.Tau_p = 100; // 2.5 100
    PAR.ekf.sigma_uv = 3; // 3 (pixels) camera measurement noise standard deviation (features)
    PAR.ekf.sigma_uv_anchor = 1; //  (pixels) camera measurement noise standard deviation (Anchors)
    PAR.ekf.visual_attitude_update = false;
    PAR.ekf.Uncertainty_depth_anchor_init = .07;
    PAR.ekf.sigma_h = .01;  // altitude update uncertanty
    PAR.ekf.sigma_cl = .025; // closing loop uncertanty
    PAR.ekf.AntGPSoffset[0] = 0; // % GPS antena to origin of robot coordinate frame, Must be expresed in NED coordiantes
    PAR.ekf.AntGPSoffset[1] = 0;
    PAR.ekf.AntGPSoffset[2] = 0;
    PAR.ekf.sigma_id_range_dem = 10; //10  // Feats with range:  sig_d_ini = depth/par.ekf.sigma_id_range_dem
    PAR.ekf.sigma_id_WOrange_dem = 5; // 5 // Feats without range:  sig_d_ini = depth/par.ekf.sigma_id_WOrange_dem
    PAR.ekf.sigma_sp = .001;  // speed measurment noise
    PAR.ekf.sigma_att_update = .000001;
    
    PAR.ekf.min_vel_start_init = .1; // m/s , Minimun x-y velocity for starting the initialization proccess of ekf feats
    PAR.ekf.min_vel_init_feats = .02; // m/s, Minimun x-y velocity for initialize new feats
    PAR.ekf.min_vel_update_feats = .02; // m/s Minimun x-y velocity for updating feats
    PAR.ekf.min_n_pred_feats_for_init = 20; // Minimun number of predicted feats in current frame, for initialize new feats using the last available range  
    PAR.ekf.max_n_feats_for_init = 100; // maximun number of feats in state for initialize new feats,

    // Mapping parameters
    PAR.map.n_consec_kf_wo_link = 10;
    PAR.map.c_consec_anchor_wo_link = 100;
    PAR.map.n_consec_kf_wo_link_match = 50; // deprecated!!  // If there is no visual link from the last KFrame to the i-Kframe for n consecutive Kframes break (stop the matching)
    PAR.map.min_kf_matched_for_keep_anchor = 3; // minumin number of keyframes that a anchor must be matched to be keeped in the global map

    // Image processing paramters
    PAR.img.image_rows = 240;
    PAR.img.image_cols = 320;
    PAR.img.n_sub_imgs = 1;
    PAR.img.search_margin = 15;
    
    // Bebop2 Camera        
        PAR.img.cam_parameters.distortions[0] = 0.02921;
        PAR.img.cam_parameters.distortions[1] = -0.00504; 
        PAR.img.cam_parameters.distortions[2] = 0.00297; 
        PAR.img.cam_parameters.distortions[3] = -0.00843; 
        PAR.img.cam_parameters.distortions[4] = 0.00000;   
        PAR.img.cam_parameters.cc[0] = 156.24435;
        PAR.img.cam_parameters.cc[1] = 117.04562 ;
        PAR.img.cam_parameters.fc[0] = 206.34225;
        PAR.img.cam_parameters.fc[1] = 268.65192;
        PAR.img.cam_parameters.alpha_c = 0;        

   
    PAR.img.check_innovation_for_mathing = false; 
    PAR.img.max_innov_pixels = 50; // if no stochastic innovation is checked then use this constant value
    PAR.img.minimun_distance_new_points = 15; // pixels

    // Sensors parameters
    PAR.sensors.range_sensor.max_range_pattern = 6; // max range of pattern, from sensor beam pattern
    PAR.sensors.range_sensor.r_max = 1.75; // maximun radius of elipsoid at (max range of pattern) from sensor beam pattern
    PAR.sensors.range_sensor.max_range_operation = 6; // max operation range of sensor (9m according to data sheet)
    PAR.sensors.range_sensor.Range_offset = 0; // offset for range measurements
    
    PAR.sensors.barometer_sensor.M = 0.0289644; // kg/mol standard molar mass of atmospheric air
    PAR.sensors.barometer_sensor.R = 8.31432; // N-m/mol-K universal gas constant for air
    PAR.sensors.barometer_sensor.a = .0065; // K/m
    PAR.sensors.barometer_sensor.g = 9.80665; // constant gravity
    PAR.sensors.barometer_sensor.T0 = 288.15; //  Kelvin standard temperature at sea level
    PAR.sensors.barometer_sensor.P0 =  101325; // N/m^2 standard pressure at sea level
    PAR.sensors.barometer_sensor.L0 = -0.0065; // K/m lapse rate of temperature deacrese in lower atmosphere
    //---------------------------------------------
    PAR.sensors.barometer_sensor.la =  1670 ; // altitude over sea level (zapopan)
    PAR.sensors.barometer_sensor.lt = 32.7633; // Temperature at flight location (celcius)

    // Close_loop process parameters
    PAR.close_loop.std_ab_vo = .1; // m
    PAR.close_loop.std_ab_cl = .1; // m
    PAR.close_loop.min_matches_for_potential_loop = 40; // Minumin matches before RANSAC to consider a potential loop
    PAR.close_loop.min_inliers_for_potential_loop = 10; // Minimun inliers after RANSAC to consider a potential loop
    PAR.close_loop.min_KeyFrames_with_potential_loop = 1; // Minimun number of (near) keyframes with potential loop detections to consider an actual loop
    PAR.close_loop.min_matches_for_computing_pos = 7; // Minimun number of matches for computing the updated pos in function "get_measured_position_of_current_KF"
    PAR.close_loop.min_time_since_last_close  = 3; // seconds // After a closure, the close loop process must wait "min_num_f_since_last_close" frames for try to detect the next closure
   
   // Plot 3D
    PAR.plot_3D.viewer_width = 1400;
    PAR.plot_3D.viewer_height = 600;
    PAR.plot_3D.default_init_axis_view = 5;
    
    PAR.plot_3D.show_grid_xy = false;
    PAR.plot_3D.grid_rect_xy.x = -20;
    PAR.plot_3D.grid_rect_xy.y = -20;
    PAR.plot_3D.grid_rect_xy.width = 40;
    PAR.plot_3D.grid_rect_xy.height = 40;    
    PAR.plot_3D.grid_xy_z = -10;

    PAR.plot_3D.show_grid_yz = true;
    PAR.plot_3D.grid_rect_yz.x = -20;
    PAR.plot_3D.grid_rect_yz.y = -20;
    PAR.plot_3D.grid_rect_yz.width = 40;
    PAR.plot_3D.grid_rect_yz.height = 40;    
    PAR.plot_3D.grid_yz_x = 0;
    PAR.plot_3D.show_frame = true;
    PAR.plot_3D.show_efk_feats = true ;
    PAR.plot_3D.show_ekf_anchors = true;
    PAR.plot_3D.show_global_map = true;
    PAR.plot_3D.show_ekf_trajectory = false;
    PAR.plot_3D.show_camera_pose = true;
    PAR.plot_3D.show_key_frames = true;


   // Control
   PAR.control.kx = 1;   // gain for x axis movements
   PAR.control.ky = 1;   // gain for y axis movements
   PAR.control.kz = 1;    // gain for x axis movements
   PAR.control.kyaw = .5;   // gain for yaw movements
   PAR.control.MaxVel_xy = .5;  // m/s    Maximun velocity allowed for x-y movements
   PAR.control.MaxVel_z = .5; // m/s Maximun velocity allowed for z (vertical) movement
   PAR.control.MaxVel_yaw = 1; // rad/s Maximun velocity allowed for rotational (yaw) movement
   PAR.control.MaxVel_xy2 = 100;  // Percentage of maximun velocity allowed for x-y movements
   PAR.control.MaxVel_z2 = 100; // percentage of maximun velocity allowed for z (vertical) movement
   PAR.control.MaxVel_yaw2 = 50; // parcentage of Maximun velocity allowed for rotational (yaw) movement
   PAR.control.Max_error_xy_reach_p = .1; // m  Maximun euclidean error allowed for considered a "go to point" command succesuful
   PAR.control.Max_error_z_reach_p = .25;
   PAR.control.Max_error_yaw_reach_p = 5*(3.1416/180); // rads  Maximun rotational (yaw) error allowed for considered a "go to point" command succesuful  
   PAR.control.home_x_a_def = .5; 
   PAR.control.home_y_a_def = .5;
   PAR.control.home_lamba = -.5;
 


} 




