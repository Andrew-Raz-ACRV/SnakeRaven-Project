#include "Raven_Controller.h"

//**************************Raven_Controller********************//
/*
This file contains functions for:
- Initialising the Raven_controller
- Setting up threads
- The ROS publisher and subscriber related functions

NOTE the other .cpp files for Raven_controller class:
- Raven_controller = where the class variables are initialised and the ROS communication occurs
- Console_process = where the robot state machine is defined
- Menu_screens = where functions for text display for the terminal are defined
- Keyboard_interactions = where functions for getting keyboard teleoperation commands are defined
- Raven_motion = where functions for robot motion are defined

author: Andrew Razjigaev 2023
*/

/**
*	\fn Raven_Controller()
*
* 	\brief this is the constructor
*
* 	\param void
*
*	\return none
*/
Raven_Controller::Raven_Controller()
{

}

/**
*	\fn initial()
*
* 	\brief initialize everything for this program.
*
* 	\param int argc, char** argv
*
*	\return void
*/

void Raven_Controller::initial(int argc, char** argv)
{
	init_sys();
	if(!init_ros(argc,argv))
  	{
     		ROS_ERROR("Fail to initialize ROS. Exiting!");
		exit(1);
  	}
	
	//print the welcome greets on console
	init_words();
}


/**
*	\fn void init_sys() 
*
*	\brief initialize default system parameter settings.
*
* 	\param void
*
*	\return void
*/

void Raven_Controller::init_sys()  
{
	//Controller settings:
	this->control_mode = 0;
	this->dx_max = 2.0; //mm
	this->tol_error = 0.1; //magnitude error

	this->motor_maxspeed_rad1 = 0.0001/2.0; //rad for larger joints //5.0
	this->motor_maxspeed_rad2 = 0.001/2.0; //rad for smaller joints
	this->motor_maxspeed_mm = 0.0001/15.0; //mm

	this->PUB_COUNT = 0;
	this->SUB_COUNT = 0;
	this->counter = 0;
	this->factor = 0;

	this->RECEIVED_FIRST = false;
	this->SHOW_STATUS = false;
	this->NO_TARGET_SET = true;
	this->GO_HOME = true;
	this->CALIBRATED = false;
	this->MANUAL = false;

	//Initialise the arrays
	for(int i = 0; i < 14; i++){
		this->delta_joint[i] = 0;
		this->jpos_initial[i] = 0;
		this->mv2jpos_rate[i] = 1;
	}

	//DEFINE LENGTH OF CUSTOM INSTRUMENTS HERE
	//Length of snake raven tool from mid-adaptor to Snake-base/RCM
	//The tube length in mm;
	//this->snake_length = 323.5; //THE FIRST TWO MODULE SNAKE BLUE
	this->right_snaketool_length = 445; //THE SECOND ENDOSCOPIC TWO MODULE SNAKE BLUE
	this->left_snaketool_length = 475;
	//NOTE default tool is 400mm long to RCM


	//Home Configuration for calibration (Perpendicular)
	//left to give way to right
	this->jpos_initial[SHOULDER_GOLD] = deg2rad(39.5967); //Move out of way deg2rad(57);
	this->jpos_initial[ELBOW_GOLD] = deg2rad(-102.0840); //Move out of way deg2rad(-59);
	this->jpos_initial[Z_INS_GOLD] = left_snaketool_length; //snake_length tube length in mm;  //Move out of way deg2rad(500);

	//right to initial pose
	this->jpos_initial[SHOULDER_GREEN] = deg2rad(-39.5967); //Move out of way deg2rad(-48);
	this->jpos_initial[ELBOW_GREEN] = deg2rad(-77.9160);    //Move out of way deg2rad(-36);
	this->jpos_initial[Z_INS_GREEN] = right_snaketool_length;			//Move out of way deg2rad(500);

	//Calibration ratios for the tool:
	//converts mv into jpos 
	//left
	this->mv2jpos_rate[Z_INS_GOLD] = -1; //up is negative
	this->mv2jpos_rate[TOOL_ROT_GOLD] = -1.17;
	this->mv2jpos_rate[WRIST_GOLD] = 0.87;
	this->mv2jpos_rate[GRASP1_GOLD] = 0.87;
	this->mv2jpos_rate[GRASP2_GOLD] = -0.89;
	//right
	this->mv2jpos_rate[Z_INS_GREEN] = -1; //up is negative
	this->mv2jpos_rate[TOOL_ROT_GREEN] = -1.50; //p pan
	this->mv2jpos_rate[WRIST_GREEN] = -1.50; //p tilt
	this->mv2jpos_rate[GRASP1_GREEN] = -3.00; //-1.65; // d pan 2.25
	this->mv2jpos_rate[GRASP2_GREEN] = 3.00; //-1.55; // d tilt //3.00 is for extreme bending!

	//Old right arm rates
	//-1.50; //good for blue tool
	//0.90; //good
	//-0.87; //was inverted for blue tool
	//-0.91; //good

	//Set these things for SnakeRaven tools:
	this->GoldSnake.initialisation(0,0,0,0,0,0,false);
	this->GreenSnake.initialisation(0.20,3,1,0.88,3,1,true); //Parameter vector
	this->GoldSnake.isrightarm = false;
	this->GreenSnake.isrightarm = true;

	//Set for teleoperation control
	this->Greenarm.Ttarget = GreenSnake.Tend;
	this->Greenarm.Tend = GreenSnake.Tend;
	this->Goldarm.Ttarget = GoldSnake.Tend;
	this->Goldarm.Tend = GoldSnake.Tend;

	//Hand Eye Calibration variables
	this->Aruco_transform = MatrixXd::Identity(4,4);
	//Cheap way of getting hand-eye calibration matrix by inspection //118 //118 + 180 + 62 = 236.001 //Latest 118+180-62+15 =250
	Affine3d Thg(Translation3d(0, 0, -2) * AngleAxisd(deg2rad(251.001), Vector3d::UnitZ()));
	this->Tend2camera = Thg.matrix();

	//Hard way from hand eye calibration:
/*	this->Tend2camera << 
	0.983941, 0153045, 0.0918522, 0.202904, 
	-0.172836, 0.945449, 0.27614, 0.202904, 
	-0.0445798, -0.287581, 0.956718, -3.45668, 
	0, 0, 0, 1;
	*/
	//this->Tend2camera = Tnorm(this->Tend2camera);
	this->Tcam2end = Tend2camera.inverse();

	//IBVS:
	this->Vc = MatrixXd::Zero(6, 1);
	this->Wc = MatrixXd::Zero(3, 1);
	this->IBVS_active = false;
	this->IBVS_STOP = false;

	//Waypoint
	this->waypoint_id = 0;
	//RCM transforms
	this->RightRCM << 0, 0, -1, -300.71, 0, 1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;
	this->LeftRCM << 0, 0, 1, 300.71, 0, -1, 0, 61, 1, 0, 0, -7, 0, 0, 0, 1;

	//Record Data set up the columns for the csv file sec,nsec and each 12 values for Tend and Ttarget transforms
	this->RECORDING = false;
	this->record_freq = 100; //hz
	if (RECORDING){
		this->pFile = fopen("SnakeRavenData.csv","w");
		fprintf(pFile, "sec, nsec, Te(0 0), Tt(0 0), Te(1 0), Tt(1 0), Te(2 0), Tt(2 0), Te(0 1), Tt(0 1), Te(1 1), Tt(1 1), Te(2 1), Tt(2 1), Te(0 2), Tt(0 2), Te(1 2), Tt(1 2), Te(2 2), Tt(2 2), Te(0 3), Tt(0 3), Te(1 3), Tt(1 3), Te(2 3), Tt(2 3), q1, q1d, q2, q2d, q3, q3d, q4, q4d, q5, q5d, q6, q6d, q7, q7d,\n");
	}

	this->RECORDING_CAMERA = true;
	if (RECORDING_CAMERA)
	{
		this->pFile = fopen("SnakeRavenCameraData.csv","w");
		fprintf(pFile, "sec, nsec, Tc(0 0), Tt(0 0), Tc(1 0), Tt(1 0), Tc(2 0), Tt(2 0), Tc(0 1), Tt(0 1), Tc(1 1), Tt(1 1), Tc(2 1), Tt(2 1), Tc(0 2), Tt(0 2), Tc(1 2), Tt(1 2), Tc(2 2), Tt(2 2), Tc(0 3), Tt(0 3), Tc(1 3), Tt(1 3), Tc(2 3), Tt(2 3), q1, q1d, q2, q2d, q3, q3d, q4, q4d, q5, q5d, q6, q6d, q7, q7d,\n");
	}
}

/**
*	\fn int init_ros(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return bool
*/
bool Raven_Controller::init_ros(int argc, char** argv) 
{
	//initialize ROS
	ros::init(argc, argv, "snake_raven_controller"); //"AutoCircle_generator"

	static ros::NodeHandle n;
	//SUBSCRIBERS:
	//Joint state!!
	RavenJoint_subscriber   = n.subscribe("joint_states", 1, &Raven_Controller::callback_joint_states,this);
	//Aruco_pose subscriber
	Arucopose_subscriber = n.subscribe("/image_converter/aruco_pose", 1, &Raven_Controller::callback_aruco_pose,this);
	//Camera_velocity subscriber
	Camera_velocity_subscriber = n.subscribe("/image_converter/camera_velocity", 1, &Raven_Controller::callback_camera_velocity,this);
	
	//PUBLISHERS:
	//Joint move!!
  	RavenJointmove_publisher = n.advertise<raven_jointmove>("raven_jointmove", 1);
  	//Robot mode
  	SnakeRaven_mode_publisher = n.advertise<std_msgs::Float32MultiArray>("snakeraven_mode", 1);

	//Recording data publisher
	Snakeraven_publisher = n.advertise<snakeraven_state>("snakeraven_state", 1);
	return true;
}

/**
*	\fn void start_thread()
*
* 	\brief start the console_thread and ros_thread
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::start_thread()
{
	pthread_create(&console_thread,NULL,Raven_Controller::static_console_process,this);
	pthread_create(&ros_thread,NULL,Raven_Controller::static_ros_process,this);

}

/**
*	\fn void join_thread()
*
* 	\brief join the console_thread and ros_thread
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::join_thread()
{
	pthread_join(console_thread,NULL);
	pthread_join(ros_thread,NULL);

	final_words();
}


/**
*	\fn void *ros_process(void)
*
* 	\brief this thread is dedicated to ros pub & sub
*
* 	\param a pointer to void
*
*	\return void
*/
void* Raven_Controller::ros_process(void)
{
	if(ros::isInitialized())
	{
		if(!RECEIVED_FIRST){
			cout<<endl<<"Waiting for the first receive of raven_state..."<<endl;
		}
		else
		{
			cout<<"First raven_state received."<<endl;
		}

		while(ros::ok() && RECEIVED_FIRST)
		{
			//cout<<"Publishing raven_automove... "<<endl;
			publish_raven_jointmove();
			publish_snakeraven_state(); //record data
			publish_snakeraven_mode();
			//output_SUBinfo();
			//output_PUBinfo();
		}

		if(RECEIVED_FIRST)
			cout<<"ros_process is shutdown."<<endl;
	}
	return 0;
}

//Publish the mode:
void Raven_Controller::publish_snakeraven_mode()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static std_msgs::Float32MultiArray msg_snake_mode;	
	msg_snake_mode.data.clear();

	msg_snake_mode.data.push_back(control_mode);
	//NOTE:
	//idle = 0
	//joint_control =1
	//calibrate = 2
	//reset = 3
	//hand eye = 4
	//IBVS_not_active = 5	
	//IBVS_active = 5.5

	//Secretly bring the teleop command for vpc
	for (int i = 0; i < 3; ++i)
	{
		msg_snake_mode.data.push_back(Vc(i));
	}

	SnakeRaven_mode_publisher.publish(msg_snake_mode);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	PUB_COUNT ++;
}

/*
//Publish teleop command for vpc:
void Raven_Controller::publish_teleop()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static std_msgs::Float32MultiArray msg_teleop;	
	msg_teleop.data.clear();

	for (int i = 0; i < 3; ++i)
	{
		msg_teleop.data.push_back(Vc(i));
	}

	teleop_publisher.publish(msg_teleop);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	PUB_COUNT ++;
}

*/

/*
// Gets Aruco pose from the image processor
*/
void Raven_Controller::callback_aruco_pose(std_msgs::Float32MultiArray msg)
{
	//Convert 12 value arrays (n o a t) to Eigen matrix 4x4:
	Aruco_transform = MatrixXd::Identity(4, 4);
	int i = 0;
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			Aruco_transform(row,col) = msg.data[i];
			i++;
		}
	}
	//cout << "Pose = " << Aruco_transform << endl;
}

/*
// Gets Camera Orientation velocity from the image processor
*/
void Raven_Controller::callback_camera_velocity(std_msgs::Float32MultiArray msg)
{
	//Convert angular velocity into Eigen vector elements:
	for (int i = 0; i < 3; i++)
	{
		Wc(i) = msg.data[i];
	}
	//cout << "Vc = " << Vc << endl;
}


//Record Function 

void Raven_Controller::publish_snakeraven_state()
{
	//We want to know the T_end and T_desired for snake raven, 
	//so convert that data and send it in a message
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static snakeraven_state msg_snakeraven_state;	

	// Put data into message
	msg_snakeraven_state.stamp = ros::Time::now(); //hdr

	//Convert Eigen matrices to 12 value arrays:
	int i = 0;
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			msg_snakeraven_state.Tend_desired[i] = Ttarget(row,col);
			msg_snakeraven_state.Tend_actual[i] = Tend(row,col);
			i++;
		}
	}
	// Record control state
	msg_snakeraven_state.control_mode = control_mode;

	// (2) send new command
	Snakeraven_publisher.publish(msg_snakeraven_state);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
}

//csv record function
void Raven_Controller::record_to_csv()
{
	//We want to know the T_end and T_desired for snake raven, 
	//Time
	fprintf(pFile, "%"PRIu32", %"PRIu32",", ros::Time::now().sec, ros::Time::now().nsec);

	//Convert Eigen matrices to 12 values:
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			fprintf(pFile, "%.4f, %.4f,", Tend(row,col), Ttarget(row,col));
		}
	}

	//Put the Joint angles in the file as well, the measured q and the q_desired:
	for (int i = 0; i < GreenSnake.DOF; i++){
		fprintf(pFile, "%.4f, %.4f,", GreenSnake.q(i), GreenSnake.q_desired(i));
	}

	//End line
	fprintf(pFile, "\n");
}

//csv record function
void Raven_Controller::record_to_csv_camera_pose()
{
	//We want to know the Tcamera and Tcam_target for snake raven, 
	//Time
	fprintf(pFile, "%"PRIu32", %"PRIu32",", ros::Time::now().sec, ros::Time::now().nsec);

	//Convert Eigen matrices to 12 values:
	for (int col = 0; col < 4; col++){
		for (int row = 0; row < 3; row++)
		{
			fprintf(pFile, "%.4f, %.4f,", Tcamera(row,col), Tcam_target(row,col));
		}
	}

	//Put the Joint angles in the file as well, the measured q and the q_desired:
	for (int i = 0; i < GreenSnake.DOF; i++){
		fprintf(pFile, "%.4f, %.4f,", GreenSnake.q(i), GreenSnake.q_desired(i));
	}

	//End line
	fprintf(pFile, "\n");
}



void * Raven_Controller::static_console_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->console_process();
}

void * Raven_Controller::static_ros_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->ros_process();
}

/**
*	\fn void publish_raven_automove()
*
*	\brief publish the auto circle command to raven_automove topic on ROS.
*
* 	\param int
*
*	\return void
*/
void Raven_Controller::publish_raven_jointmove()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_jointmove msg_raven_jointmove;	

	// (1) wrap up the new command	
	//msg_raven_jointmove.hdr.stamp = msg_raven_jointmove.hdr.stamp.now(); //hdr

	//JOINT POSITION PUBLISH
	for (int i = 0; i<14; i++){
		msg_raven_jointmove.delta_joint[i] = delta_joint[i];
		//Reset delta_array to 0 for next loop
		delta_joint[i] = 0;
	}

	// (2) send new command
	RavenJointmove_publisher.publish(msg_raven_jointmove);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	PUB_COUNT ++;
}

/**
*	\fn void autoRavenStateCallback(raven_2::raven_state msg)
*
*	\brief This function is automatically called whenever someone publish to raven_state topic
*
* 	\param raven_2::raven_state msg
*
*	\return void
*/
void Raven_Controller::callback_joint_states(sensor_msgs::JointState msg)
{
	// Get Joint Data from joint state
	//Reading joints like this: 0-6 left, 7-13 right
	//cout<<"Shoulder gold = "<<msg.position[0]<<endl;

	//(1) Copy joint state to local variable in radians and mm
	for (int i = 0; i<14; i++){
		if ((i==Z_INS_GOLD)||(i==Z_INS_GREEN)){
			jpos_current[i] = msg.position[i]; //mm
		}
		else{
			jpos_current[i] = deg2rad(msg.position[i]); //radians
		}
	}

	// (2) update recieved data count
	SUB_COUNT ++;
	RECEIVED_FIRST = true;
}
/**
*	\fn void output_PUBinfo()
*
* 	\brief shows the publish status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_PUBinfo()
{
	cout<<"snake_raven_controller publish: raven_jointmove["<<PUB_COUNT<<"]"<<endl;

	//Raven joint positions
		//Joint position update:
	for (int i=0; i<2; i++){
		//per arm
		if(i==0){
			cout<<"\n\tLEFT ARM delta_joint = "<<endl;	
			for (int j=0; j<7; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<delta_joint[i*SHOULDER_GREEN + j]<<endl;
			}
		}
		else if(i==1){
			cout<<"\n\tRIGHT ARM data1.position = "<<endl;
			for (int j=0; j<7; j++){
				//Per DOF
				cout<<"\tDOF "<<j<<" = "<<delta_joint[i*SHOULDER_GREEN + j]<<endl;
			}
		}
	}
	cout<<endl;
	cout<<endl;
	//ROS_INFO("snake_raven_controller publish: raven_jointmove[%d]", PUB_COUNT);
}

/**
*	\fn void output_SUBinfo()
*
* 	\brief shows the subscribe status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_SUBinfo()
{
	cout<<"snake_raven_controller subscribe: joint_states["<<SUB_COUNT<<"]"<<endl;

	//Raven joint positions
	int value;
	//Joint position update:
	for (int i=0; i<2; i++){
		//per arm
		if(i==0){
			cout<<"\n\tLEFT_ARM.position = "<<endl;	
			for (int j=0; j<7; j++){
				//Per DOF
				value = i*SHOULDER_GREEN + j;

				if (value==Z_INS_GOLD){
					cout<<"\tDOF "<<j<<" = "<<jpos_current[value]<<endl;
				}
				else{
					cout<<"\tDOF "<<j<<" = "<<rad2deg(jpos_current[value])<<endl;
				}
			}
		}
		else if(i==1){
			cout<<"\n\tRIGHT_ARM.position = "<<endl;
			for (int j=0; j<7; j++){
				//Per DOF
				value = i*SHOULDER_GREEN + j;

				if (value==Z_INS_GREEN){
					cout<<"\tDOF "<<j<<" = "<<jpos_current[value]<<endl;
				}
				else{
					cout<<"\tDOF "<<j<<" = "<<rad2deg(jpos_current[value])<<endl;
				}
			}
		}
	}
	cout<<endl;
	cout<<endl;
}

