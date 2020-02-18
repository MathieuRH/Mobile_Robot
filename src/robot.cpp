

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <sensor_range.h>


using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
// Robot::Robot gives access to the function Robot in the robot class
// This funtion modifies the "protected" parameters (pose)of the robot (x,y,theta) and its name
// to initialize the initial position of the robot 
// Arguments are passed by value : that means that the passed arguments cannot be modified 
//while definig a new robot

{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
//We don't need this function to be available from outside the Robot class anymore
//Because we have a better and saffer way to move the robot with the Robot::moveVW method
//It should not be available for external use (we put this function in the "private" section of the class Robot in robot.h) 
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double left, double right,bool wheels_init_)
// we add the boolean wheels_init_ to the parameters of the function in order to check 
//if the initialization of the wheels has been done
{
	double v, omega, x_dot,y_dot;
	
	if (wheels_init_){
		
		int absl,absr;
		absl=fabs(left);
		absr=fabs(right);
		double a=max(absl/velocity_limit,absr/velocity_limit);
		
        if (a<1){
			a = 1;
		}
		else{
			cout<<"Error : the applied velocities are outside the bounds"<<endl;
		}	
		double _left=left/a;
		double _right=right/a;
		
		v=r*(_left+_right)/2;
		omega=r*(_left-_right)/(2*b);
			
		x_dot=v*cos(pose_.theta);  
		y_dot=v*sin(pose_.theta);
			
		moveXYT(x_dot,y_dot,omega);
		
	}
	else {
		cout<<"Please initialize the radius of the wheels and the base distance"<<endl;
	}
	
}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega,bool wheels_init_)
{
    //double x_dot,y_dot;
    //x_dot=_v*cos(pose_.theta);
    //y_dot=_v*sin(pose_.theta);
	
	//moveXYT(x_dot,y_dot,_omega); //For the former method
	
	// function changed following part 2.3
	double _left, _right;
	_left=(_v+b*_omega)/r;
	_right= (_v-b*_omega)/r;
	rotateWheels(_left,_right,wheels_init_);
	
}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p,bool wheels_init_)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);


    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0),wheels_init_);
}


void Robot::moveWithSensor(Twist _twist,bool wheels_init_)
{

    //call all the sensors of the robot 
    int n=sensors_.size();
    for (int i=0;i<n;i++){
        sensors_[i]->updateFromRobotPose(pose());  // for each sensors the closest wall is stored
		//in the s_ variable
        //cout<<"s : "<<Rsensors_[i]->read()<<endl;

        sensors_[i]->correctRobotTwist(_twist);
    }
    double v,omega;
    v=_twist.vx;
    omega=20*_twist.vy + _twist.w;
    moveVW(v,omega,wheels_init_);

}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

//Wheels : radius and base distance initialization
 void Robot::initWheel(double _r,double _b, bool &init,double _velocity_limit){
	 r=_r;
	 b=_b;
	 velocity_limit=_velocity_limit;
	 init=true;
}


