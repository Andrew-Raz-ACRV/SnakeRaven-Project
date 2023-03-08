/*
This is the listener.cpp it is a test script that subscribes to raven_jointmove and publishes raven_state

*/

#include <string.h>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>
//ROS messages
#include "raven_2/raven_state.h"
#include "snake_raven_controller/raven_jointmove.h"
#include "raven_2/DS1.h" //struct param_pass is defined here.

#define ROS_PUBLISH_RATE 100
#define LEFT_ARM 0  //is also the GOLD_ARM
#define RIGHT_ARM 1 //is also the GREEN_ARM

//The joint labels:
//LEFT
#define SHOULDER_GOLD 0
#define ELBOW_GOLD 1
#define Z_INS_GOLD 2
#define TOOL_ROT_GOLD 4
#define WRIST_GOLD 5
#define GRASP1_GOLD 6
#define GRASP2_GOLD 7
//RIGHT
#define SHOULDER_GREEN 8
#define ELBOW_GREEN 9
#define Z_INS_GREEN 10
#define TOOL_ROT_GREEN 12
#define WRIST_GREEN 13
#define GRASP1_GREEN 14
#define GRASP2_GREEN 15


using namespace raven_2;
using namespace std;
using namespace snake_raven_controller;

//---------------------------global variables---------------------------
ros:: Subscriber sub_jointmove;
ros:: Publisher pub_ravenstate;

raven_2::raven_state CURR_RAVEN_STATE;

int PUB_COUNT;
int SUB_COUNT;
static struct param_pass data1;

float array[16];

float start[16];

tf::Quaternion Q_ori[2];

pthread_t rt_thread;

//-------------------------function declaration-------------------------
void autoincrCallback(raven_jointmove);
int init_ravenstate_publishing(ros::NodeHandle &);
int init_ros(int, char**);
void init_sys(int, char**);
void init_raven();

void output_STATUS();

void publish_raven_state_ros();
void *rt_process(void*);

//---------------------------------main---------------------------------
int main(int argc, char **argv)
{
	init_sys(argc,argv);
	
	pthread_create(&rt_thread,NULL,rt_process,NULL);

	ros::spin();
	return 0;
}

//--------------------------function definition--------------------------
/**
*	\fn void *rt_process(void *)
*
* 	\brief this thread is dedicated to ros publishing of raven_state data
*
* 	\param a pointer to void
*
*	\return void
*/
void *rt_process(void*)
{
	if(ros::isInitialized())
	{
		ros::Time time;
		time = time.now();
		while(ros::ok())
		{
			publish_raven_state_ros();
			if((time.now()-time).toSec() > 1)
			{
				output_STATUS();
				time = time.now();
			}
		}
		cout<<"rt_process is shutdown."<<endl;
		return( NULL);
	}
	else
		return 0;
}



/**
*	\fn void autoincrCallback(raven_2::raven_automove msg)
*
*	\brief This function is automatically called whenever someone publish to the raven_automove topic
*
* 	\param raven_2::raven_automove msg
*
*	\return void
*/
void autoincrCallback(raven_jointmove msg) //this was in local_io.cpp
{
	//move msg value into data1 

	//Joint position update: += is a delta

	//GOLD arm
	data1.jpos_d[SHOULDER_GOLD] += msg.delta_joint[SHOULDER_GOLD];
	data1.jpos_d[ELBOW_GOLD] += msg.delta_joint[ELBOW_GOLD];
	data1.jpos_d[Z_INS_GOLD] += msg.delta_joint[Z_INS_GOLD];
	data1.jpos_d[TOOL_ROT_GOLD] += msg.delta_joint[TOOL_ROT_GOLD];
	data1.jpos_d[WRIST_GOLD] += msg.delta_joint[WRIST_GOLD];
	data1.jpos_d[GRASP1_GOLD] += msg.delta_joint[GRASP1_GOLD];
	data1.jpos_d[GRASP2_GOLD] += msg.delta_joint[GRASP2_GOLD];


	//Green arm
	data1.jpos_d[SHOULDER_GREEN] += msg.delta_joint[SHOULDER_GREEN];
	data1.jpos_d[ELBOW_GREEN] += msg.delta_joint[ELBOW_GREEN];
	data1.jpos_d[Z_INS_GREEN] += msg.delta_joint[Z_INS_GREEN];
	data1.jpos_d[TOOL_ROT_GREEN] += msg.delta_joint[TOOL_ROT_GREEN];
	data1.jpos_d[WRIST_GREEN] += msg.delta_joint[WRIST_GREEN];
	data1.jpos_d[GRASP1_GREEN] += msg.delta_joint[GRASP1_GREEN];
	data1.jpos_d[GRASP2_GREEN] += msg.delta_joint[GRASP2_GREEN];


	SUB_COUNT ++;
}


/**
*	\fn int init_ravenstate_publishing(ros::NodeHandle &n) 
*
*	\brief initialize the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return int
*/
int init_ravenstate_publishing(ros::NodeHandle &n)  //this was in local_io.cpp
{

	pub_ravenstate = n.advertise<raven_state>("ravenstate",1);
	sub_jointmove = n.subscribe<raven_jointmove>("raven_jointmove",1,autoincrCallback,ros::TransportHints().unreliable());

}


/**
*	\fn int init_sys(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return void
*/
void init_sys(int argc, char** argv)
{
	init_raven();

	if(init_ros(argc,argv))
	{
		cerr<<"ERROR! Fail to init ROS. Exiting.\n";
		exit(1);
	}

	cout<<"Welcome to the listener node for RAVEN2"<<endl;
	cout<<"I'm waiting for the talker..."<<endl;
	 
}


/**
*	\fn int init_sys() 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param void
*
*	\return void
*/
void init_raven()
{

	//Assume this is our raven init state
	start[SHOULDER_GOLD] = 1;
	start[ELBOW_GOLD] = 2;
	start[Z_INS_GOLD] = 3;
	start[TOOL_ROT_GOLD] = 4;
	start[WRIST_GOLD] = 5;
	start[GRASP1_GOLD] = 6;
	start[GRASP2_GOLD] = 7;

	//Green arm
	start[SHOULDER_GREEN] = 8;
	start[ELBOW_GREEN] = 9;
	start[Z_INS_GREEN] = 10;
	start[TOOL_ROT_GREEN] = 11;
	start[WRIST_GREEN] = 12;
	start[GRASP1_GREEN] = 13;
	start[GRASP2_GREEN] = 14;

	//..
	data1.jpos_d[SHOULDER_GOLD] = start[SHOULDER_GOLD];
	data1.jpos_d[ELBOW_GOLD] = start[ELBOW_GOLD];
	data1.jpos_d[Z_INS_GOLD] = start[Z_INS_GOLD];
	data1.jpos_d[TOOL_ROT_GOLD] = start[TOOL_ROT_GOLD];
	data1.jpos_d[WRIST_GOLD] = start[WRIST_GOLD];
	data1.jpos_d[GRASP1_GOLD] = start[GRASP1_GOLD];
	data1.jpos_d[GRASP2_GOLD] = start[GRASP2_GOLD];

	//Green arm
	data1.jpos_d[SHOULDER_GREEN] = start[SHOULDER_GREEN];
	data1.jpos_d[ELBOW_GREEN] = start[ELBOW_GREEN];
	data1.jpos_d[Z_INS_GREEN] = start[Z_INS_GREEN];
	data1.jpos_d[TOOL_ROT_GREEN] = start[TOOL_ROT_GREEN];
	data1.jpos_d[WRIST_GREEN] = start[WRIST_GREEN];
	data1.jpos_d[GRASP1_GREEN] = start[GRASP1_GREEN];
	data1.jpos_d[GRASP2_GREEN] = start[GRASP2_GREEN];

}


/**
*	\fn int init_ros(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return int
*/
int init_ros(int argc, char** argv)  //this was in rt_process_preempt.cpp
{
	//initialize counter
	PUB_COUNT = 0;
	SUB_COUNT = 0;

	//initialize ros 
	ros::init(argc,argv,"r2_control",ros::init_options::NoSigintHandler);
	
	//establish ros pub & sub relation
	static ros::NodeHandle n;
	init_ravenstate_publishing(n);
	
	return 0;
}


/**
*	\fn int output_STATUS()
*
*	\brief Display the recieved data from the Auto Circle Generator.
*
* 	\param void
*
*	\return void
*/
void output_STATUS()
{
	cout<<endl<<endl;
	ROS_INFO("listenerSnakeRaven publish: raven_state[%d]", PUB_COUNT);
	ROS_INFO("listenerSnakeRaven subscribe: raven_jointmove[%d]", SUB_COUNT);

	//Joint position update:
	for (int i=0; i<2; i++){
		//per arm
		if(i==0){
			cout<<"\n\tLEFT ARM data1.jpos_d = "<<endl;	
			for (int j=0; j<8; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<data1.jpos_d[i*SHOULDER_GREEN + j]<<endl;
			}
		}
		else if(i==1){
			cout<<"\n\tRIGHT ARM data1.jpos_d = "<<endl;
			for (int j=0; j<8; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<data1.jpos_d[i*SHOULDER_GREEN + j]<<endl;
			}
		}
	}
}


/**
*	\fn void publish_raven_state_ros()
*
*	\brief publish the auto circle command to raven_automove topic on ROS.
*
* 	\param void
*
*	\return void
*/
void publish_raven_state_ros()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_state msg_raven_state;

	//Insert Left joint position into raven_state
	msg_raven_state.jpos[SHOULDER_GOLD] = data1.jpos_d[SHOULDER_GOLD];
	msg_raven_state.jpos[ELBOW_GOLD] = data1.jpos_d[ELBOW_GOLD];
	msg_raven_state.jpos[Z_INS_GOLD] = data1.jpos_d[Z_INS_GOLD];
	msg_raven_state.jpos[TOOL_ROT_GOLD] = data1.jpos_d[TOOL_ROT_GOLD];
	msg_raven_state.jpos[WRIST_GOLD] = data1.jpos_d[WRIST_GOLD];
	msg_raven_state.jpos[GRASP1_GOLD] = data1.jpos_d[GRASP1_GOLD];
	msg_raven_state.jpos[GRASP2_GOLD] = data1.jpos_d[GRASP2_GOLD];

	//Insert Green joint position into raven_state
	msg_raven_state.jpos[SHOULDER_GREEN] = data1.jpos_d[SHOULDER_GREEN];
	msg_raven_state.jpos[ELBOW_GREEN] = data1.jpos_d[ELBOW_GREEN];
	msg_raven_state.jpos[Z_INS_GREEN] = data1.jpos_d[Z_INS_GREEN];
	msg_raven_state.jpos[TOOL_ROT_GREEN] = data1.jpos_d[TOOL_ROT_GREEN];
	msg_raven_state.jpos[WRIST_GREEN] = data1.jpos_d[WRIST_GREEN];
	msg_raven_state.jpos[GRASP1_GREEN] = data1.jpos_d[GRASP1_GREEN];
	msg_raven_state.jpos[GRASP2_GREEN] = data1.jpos_d[GRASP2_GREEN];


	pub_ravenstate.publish(msg_raven_state);
	
	ros::spinOnce();
	

	loop_rate.sleep();
	PUB_COUNT ++;
}
