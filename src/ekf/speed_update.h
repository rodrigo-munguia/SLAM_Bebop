#ifndef SPD_U_H
#define SPD_U_H


#include <armadillo>
#include "../parameters.h"
#include "../getData.h"

//-----------------------------------------------------------------------------
//   Speed Update
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
typedef std::vector<double> vec_t;

void speed_Update(arma::vec& x, arma::mat& P,SPD &speed,parameters &par);





#endif