#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>

namespace arpro {

class RangeSensor : public Sensor
{
public :
RangeSensor ( Robot & _robot , double _x , double _y , double _theta ) :
Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{}
// the RangeSensor constructor does nothing more



// update from current robot pose
  // void updateFromRobotPose(const Pose &_p);

// update from current sensor pose
   void update(const Pose &_p) ;
    
    // correct twist in sensor frame
   void correctTwist(Twist &_v) ;
    
   //void correctRobotTwist(Twist &_v);
   
   inline static void setEnvironment(Environment &_envir) {envir_ = &_envir;}

   inline const Pose& getPose() const {return pose_;}

   //~RangeSensor() {}

};

}
#endif // SENSOR_RANGE_H
