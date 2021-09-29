
#include "ekf.h"
#include <math.h>
#include <opencv2/viz.hpp>


bool EKF::System_init(DATA &dat)
{
    

    bool init = system_init(x,P,PAR,dat,yaw_at_home,Init_cam_position);

    if (init == true)
    {
        FeatsDATA.clear();
        AnchorsDATA.clear(); 
        Initialized = true;       
        return true;
    }
    else
    {   
        Initialized = false;
        return false;     
    }    

}

void EKF::prediction(double delta_t)
{

    prediction_state(x,P,PAR,delta_t);

}

void EKF::visual_update(FRAME *frame,GMAP &gmap,LOOP &cloop,LOCKS &locks)
{
    
    double v = sqrt(x(10)*x(10) + x(11)*x(11)); // current velocity over the x-y plane
    static double last_range = 0;
    MatchInfo Mi;
    
    if (FeatsDATA.size() > 0)
    {  
        //if (v > .05) 
            visual_delete_feats(x, P,PAR,FeatsDATA, AnchorsDATA  );
        
        if((PAR.sys.EKF_initialize_anchors == true))
            visual_init_anchors(x, P,PAR, FeatsDATA, AnchorsDATA,gmap,locks );        

            Mi = visual_match_feats(x,P,PAR,frame,FeatsDATA,AnchorsDATA,gmap,cloop,locks);

         
        visual_update_f(x,P,PAR,FeatsDATA,AnchorsDATA);

    }

    if (frame->range > 0) // if range measurement is associated with the frame
    {   

        if ((FeatsDATA.size() == 0)&&(v > PAR.ekf.min_vel_start_init))
        {  
            // add features for the first time
            visual_init_w_range(x,P,PAR,frame,FeatsDATA,AnchorsDATA); // for the first time
        }
        if ((FeatsDATA.size()>0)&&(FeatsDATA.size() < PAR.ekf.max_n_feats_for_init)&& v > PAR.ekf.min_vel_init_feats)            
        {  
          // initialize new features  
           visual_init_w_range(x,P,PAR,frame,FeatsDATA,AnchorsDATA);
        }
        last_range = frame->range;
    }
    else
    {
        if(FeatsDATA.size()>0 && Mi.n_predicted_img_feats < PAR.ekf.min_n_pred_feats_for_init && last_range >0 && v > PAR.ekf.min_vel_init_feats)
        {   
            // if the number of predicted feats to appear in the image, initialize new feats using the last range as initial depth
            frame->range = last_range;
            visual_init_w_range(x,P,PAR,frame,FeatsDATA,AnchorsDATA);   

        }
    }
    
    

}

void EKF::closing_loop_position_update(arma::vec::fixed<2> xy_pos)
{
    //this->AnchorsDATA.clear(); 
    //cl_position_update(x, P,xy_pos,PAR);
    cl_position_update2(x, P,xy_pos,PAR,FeatsDATA,AnchorsDATA);
}


void EKF::altitude_update_bar(BAR &bar)
{

   Altitude_Update_bar(x,P,bar,PAR);
    
}
void EKF::altitude_update_alt(ALT &alt)
{

    Altitude_Update_alt(x,P,alt,PAR);
}

void EKF::speed_update(SPD &speed)
{
    speed_Update(x,P,speed,PAR);
}

void EKF::attitude_update(ATT &att)
{
    //Attitude_Update(x,P,att,PAR,yaw_at_home);

  // Attitude_Update_2(x,P,att,PAR,yaw_at_home,Init_cam_position);

   Attitude_Update_3(x,P,att,PAR,yaw_at_home,Init_cam_position);
}

void EKF::store_data_for_plot(LOCKS &locks,FRAME *frame)
{   
    
   STORE_data_for_plot(x,PAR,locks,store,FeatsDATA,AnchorsDATA,frame);


}



void EKF::store_gps(GPS &gps)
{  
  
  Store_gps(gps,PAR,store);

}