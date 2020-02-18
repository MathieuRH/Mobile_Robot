#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;


int main(int argc, char **argv)
{
  bool wheels_init_=false;
  bool wheels_init_2=false;

  // default environment with moving target
  Environment envir;
 

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  Robot robot2("C3PO", 0,0,0);
 
  // Wheels initialization
  robot.initWheel(0.07,0.3,wheels_init_,10); // on modifie la fonction initWheels pour qu'elle 
  // renvoie vrai sur wheels_init_ quand l'initialisation est faite 
  robot2.initWheel(0.05,0.3,wheels_init_2,10);
  
  //Range sensor

  Pose trs = Pose(0.1,0,0); // Position du capteur dans le repère du robot
  trs = trs.transformDirect(robot.pose()); // Position du capteur dans le repère cartésien
  RangeSensor rangesensor(robot, trs.x, trs.y, trs.theta); // Ajout d'un capteur
  RangeSensor::setEnvironment(envir);

  Pose trs2 = Pose(0,0.05,0); // Position du capteur dans le repère du robot
  trs2 = trs2.transformDirect(robot.pose()); // Position du capteur dans le repère cartésien
  RangeSensor rangesensor2(robot, trs2.x, trs2.y, trs2.theta); // Ajout d'un capteur
  

  RangeSensor::setEnvironment(envir);

  
  Pose trs3 = Pose(1,1,0); // Position du capteur dans le repère du robot
  trs2 = trs2.transformDirect(robot.pose()); // Position du capteur dans le repère cartésien
  RangeSensor rangesensor3(robot, trs3.x, trs3.y, trs3.theta); // Ajout d'un capteur
   // sensors gets measurements from this environment
  RangeSensor::setEnvironment(envir);
  
  
  //Creation of the Range sensor pointers 
  RangeSensor *RSpointer=&rangesensor;
  RangeSensor *RSpointer2=&rangesensor2;
  RangeSensor *RSpointer3=&rangesensor3;

  //Creation of the bearing sensors
  Pose trsRB = Pose(0.1,0,0); // Position du capteur dans le repère du robot
  trsRB = trsRB.transformDirect(robot2.pose()); // Position du capteur dans le repère cartésien
  SensorBearing BearingSensor(robot2, trsRB.x, trsRB.y, trsRB.theta); // Ajout d'un capteur

  //Creation of the Bearing sensor pointers
  SensorBearing *SBpointer=&BearingSensor;


  //The sensors are all attached to the robot
  robot.attach(RSpointer);
  robot.attach(RSpointer2);
  robot.attach(RSpointer3);

  robot2.attach(SBpointer);
  
  envir.addRobot(robot);
  envir.addRobot(robot2);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target(),wheels_init_);
    robot2.moveWithSensor(Twist(0.4,0,0),wheels_init_2);

  }

  // plot trajectory
  envir.plot();

}
