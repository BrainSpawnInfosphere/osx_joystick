/**
 * 
 */

//--- C++ ---------------------
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//--- Boost --------------------
#include <boost/program_options.hpp>     // command line options
namespace po = boost::program_options;

//--- ROS ---------------------
#include <ros/ros.h>

//--- Joystick ----------------
#include "JoyStick.h"


int main(int argc, char *argv[]){

	// init ROS
	ros::init(argc, argv, "joystick");
	
	int joy_num = 0;
	float hz = 30.0;
	
	//--- Handle commandline options ---------------------------------------------
	po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("num", po::value<int>(),"which joystick to use, default is 0")
        ("hz", po::value<float>(), "polling frequency in Hz, default is 30 Hz")
        ;
    po::variables_map vm;       
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);   
    
    if (vm.count("help")) {
        //std::cout << "rosrun ahrs ahrs [option] \n";
        //std::cout << "    default topic in [imu] out [imu_out] \n";
        std::cout << desc << "\n";
        return 0;
    }
    
    if (vm.count("num")){
        joy_num = vm["num"].as<int>();
    }
    
    if (vm.count("hz")){
        hz = vm["hz"].as<float>();
    }
	
	// init the GLFW library
    if( !glfwInit() ){
        ROS_ERROR("Failed to initialize GLFW" );
        exit(1);
    }
    //-------------------------------------------------------------------------------
    
    TwistJoyStick js(joy_num);
    js.setUpPublisher();
    js.spin(hz);

    // Terminate GLFW
    glfwTerminate();

    // Exit program
    exit(0);
}

