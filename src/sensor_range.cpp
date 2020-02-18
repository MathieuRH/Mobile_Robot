
#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <sensor_range.h>

using namespace arpro;
using namespace std;

//Environment* Sensor::envir_ = nullptr;



//void RangeSensor::updateFromRobotPose(const Pose &_p)
 //   {
  //      cout<<"update from robot pose"<<endl;
  //      const Pose posetransform=pose_.transformDirect(_p);
  //      cout<<"pose"<<posetransform.x<<endl;
  //      RangeSensor::update(pose_.transformDirect(_p));
   //     cout<<"on a appelé update"<<endl;
 //   }

void RangeSensor::update(const Pose &_p) 
{
        Pose p1 , p2 ;
		double xs, ys, thetas, min;
		vector<Pose> Walls;
		Walls=envir_->Environment::getWalls();
		xs=_p.x;
		ys=_p.y;
		thetas=_p.theta;
        min=1000;

        for ( int i=0; i <  Walls.size ();++ i ){
            p2 = Walls[i];
            p1 = Walls[(i+1)% Walls.size()];
            double x1, x2, y1,y2,d;
            x1=p1.x;
            x2=p2.x;
            y1=p1.y;
            y2=p2.y;
            double num;
            double den;
            num=x1*y2-x1*ys-x2*y1+x2*ys+xs*y1-xs*y2;
            den=x1*sin(thetas)-x2*sin(thetas)-y1*cos(thetas)+y2*cos(thetas);
            if (den!=0){
                d=num/den;
                if (d>=0 && d<min){
                    min=d;
                }
            }
        }
        s_=min;
        cout<<"minimum distance "<<min<<endl;


}
// The given twist is expressed in the sensor frame 
// cheks if the given Twist is fine with the current sensor measurement 
void RangeSensor::correctTwist(Twist &_v) {
	double g,sm;
    g=0.9;
    sm=0.4;
    cout<<"s=sm : "<<s_-sm<<"vitesse : "<<_v.vx<<endl;
	if (_v.vx>g*(s_-sm)){
		_v.vx=g*(s_-sm);
        cout<<"corrigé fdp"<<endl;

	}
	
}


// correct twist in robot frame
//void RangeSensor::correctRobotTwist(Twist &_v)
//{

	// twist in sensor frame
//	_v = _v.transformInverse(pose_);


	// check twist in sensor frame
//	correctTwist(_v);


	// back to robot frame
//	_v = _v.transformDirect(pose_);

//}
