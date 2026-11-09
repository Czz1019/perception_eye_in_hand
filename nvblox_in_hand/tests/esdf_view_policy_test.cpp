#include "nvblox_in_hand/esdf_view_policy.hpp"
#include <cassert>
#include <limits>
#include <cmath>
int main(){
 using in_hand::observedDistance;
 assert(!observedDistance(false,false,0,.02,.12));
 assert(!observedDistance(true,false,-1,.02,.12));
 assert(!observedDistance(true,false,std::numeric_limits<float>::quiet_NaN(),.02,.12));
 assert(!observedDistance(true,false,100,.02,.12));
 assert(!observedDistance(true,false,1,0,.12));
 assert(std::abs(*observedDistance(true,false,4,.02,.12)-.04)<1e-6);
 assert(std::abs(*observedDistance(true,true,4,.02,.12)+.04)<1e-6);
 assert(observedDistance(true,false,0,.02,.12).has_value());
}
