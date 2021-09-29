
#ifndef SYS_INT_H
#define SYS_INT_H




#include <armadillo>
#include "../parameters.h"
#include "../Transforms/Euler_to_Ra2b.h"
#include "../Transforms/Ra2b_TO_Quat_a2b.h"
#include "../getData.h"
#include "ekf_types.h"



//-----------------------------------------------------------------------------
//   State vector and Covariance matrix initialization
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

bool system_init(arma::vec& x, arma::mat& P, parameters &par ,DATA &dat, double &yaw_at_home, CamPose &Init_cam_position );


#endif