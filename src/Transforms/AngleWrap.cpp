
#include "AngleWrap.h"


void AngleWrap(double &a)
{
   if( a > 3.1416)
   {
       a = a- 2*3.1416;
   }
   if( a < -3.1416)
   {
       a = a + 2*3.1416;
   }
   if (a == 3.1416)
   {
       a = a;
   }   
}