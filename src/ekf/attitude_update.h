#ifndef ATT_U_H
#define ATT_U_H


#include <armadillo>
#include "../parameters.h"
#include "../getData.h"
#include "../Transforms/Euler_to_Ra2b.h"
#include "../Transforms/Ra2b_TO_Quat_a2b.h"
#include "../Transforms/quat2R.h"
#include "ekf_types.h"

//-----------------------------------------------------------------------------
//   
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
typedef std::vector<double> vec_t;

void Attitude_Update(arma::vec& x, arma::mat& P,ATT &att,parameters &par,double &yaw_at_home);

void Attitude_Update_2(arma::vec& x, arma::mat& P,ATT &att,parameters &par,double &yaw_at_home, CamPose &Init_cam_position );

void Attitude_Update_3(arma::vec& x, arma::mat& P,ATT &att,parameters &par,double &yaw_at_home, CamPose &Init_cam_position );



#endif