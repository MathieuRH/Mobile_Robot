#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>

namespace arpro {

class SensorBearing : public Sensor
{
public :
SensorBearing ( Robot & _robot , double _x , double _y , double _theta ) :
Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{}

   void update(const Pose &_p){
       // look for first other robot
       //inline const std::vector<Robot*>& getRobots() const {return robots_;}
       std::vector<Robot*> robots;
       robots=envir_->getRobots();
       for ( auto other : robots ){
            if ( other != robot_ )
            {

                double xr, yr, xs, ys, thetas;
                xs=_p.x;
                ys=_p.y;
                thetas=_p.theta;
                xr=other->pose().x;
                yr=other->pose().y;
                double angle=atan2(yr-ys,xr-xs)-thetas;
                s_ = fmod(angle, M_PI);
                break ;
            }
       }
   }
   void correctTwist(Twist &_v){
       double g=10;
       _v.w=_v.w-g*s_;
   }



};


}
#endif
