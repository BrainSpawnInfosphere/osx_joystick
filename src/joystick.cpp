/**
 * 
 */

//--- C++ ---------------------
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//--- GLFW --------------------
#include <GL/glfw.h>

#include <boost/program_options.hpp>     // command line options
namespace po = boost::program_options;

//--- ROS ---------------------
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/**
 * JoyStick uses GLFW library to read any joystick connected to the system.
 * The joysticks are numbered from GLFW_JOYSTICK_1 (or 0) to GLFW_JOYSTICK_LAST
 * (or 16). The class defaults to the first one (0).
 */
class JoyStick {
public:
    JoyStick(int i=GLFW_JOYSTICK_1){
        joy_num = i;
        int err = glfwGetJoystickParam(joy_num,GLFW_PRESENT);
        
        if(err == GL_FALSE){
            ROS_ERROR("Couldn't connect to joystick[%i]",joy_num);
            exit(1);
        }
        
        num_axes = glfwGetJoystickParam(joy_num,GLFW_AXES);
        num_buttons = glfwGetJoystickParam(joy_num,GLFW_BUTTONS);
        
        if(num_axes == 0 || num_buttons == 0){
            ROS_ERROR("Couldn't connect to joystick[%i]",joy_num);
            exit(1);
        }
        
        //ros::NodeHandle n("~");
        ros::NodeHandle n;
        char joy_name[32];
        sprintf(joy_name, "joy%d",joy_num);
        joy_pub = n.advertise<sensor_msgs::Joy>(joy_name, 50);
        
        ROS_INFO("==================================");
        ROS_INFO("%s: axes[%i] buttons[%i]",joy_name,num_axes,num_buttons);
        ROS_INFO("==================================");
        
    }
    
    // grab the current axes and button values
    bool get(void){
        int err = 0;
        //ROS_INFO("loop");
        
        err = glfwGetJoystickPos(joy_num, a, num_axes );
        if(err == 0 || err < num_axes){
            ROS_ERROR_THROTTLE(1.0,"Couldn't read axes");
            return false;
        }
        
        err = glfwGetJoystickButtons(joy_num, b, num_buttons);
        if(err == 0 || err < num_buttons){
            ROS_ERROR_THROTTLE(1.0,"Couldn't read buttons");
            return false;
        }
        
        return true;
    }
    
    void spin(float hz){
        ros::Rate r(hz);
        unsigned int err_cnt = 0;

        // Main Loop -- go until ^C terminates
        while (ros::ok()){
            bool ok = get();
            
            // if we get enough errors, then exit
            if(!ok){ 
                ++err_cnt;
                if(err_cnt > hz*5){
                    ROS_ERROR("Can't connect to joystick ... exiting");
                    return;
                }
            }
            
            
            // publish joystick message
            sensor_msgs::Joy joy_msg;
            joy_msg.header.stamp = ros::Time::now();
            
            // copy axes
            joy_msg.axes.resize(num_axes);
            for(int i=0;i<num_axes;++i) joy_msg.axes[i] = a[i];
            
            // copy buttons
            joy_msg.buttons.resize(num_buttons);
            for(int i=0;i<num_buttons;++i) joy_msg.buttons[i] = b[i];
            
            joy_pub.publish(joy_msg);

            ros::spinOnce();
            r.sleep();
        }
	}
    
    int joy_num;
    int num_axes;
    int num_buttons;
    float a[32];
    unsigned char b[32];
    
    //sensor_msgs::Joy joy_msg;
	ros::Publisher joy_pub;
};
/*
void joystick( void )
{
    float joy1pos[ 4 ];

    // Get joystick X & Y axis positions
    glfwGetJoystickPos( GLFW_JOYSTICK_1, joy1pos, 4 );
    //glfwGetJoystickPos( GLFW_JOYSTICK_2, joy2pos, 2 );
    
    printf("J1: %f %f\n",joy1pos[0],joy1pos[1]);
    printf("J2: %f %f\n",joy1pos[2],joy1pos[3]);
}
*/
/*
void printJS(int i){
    int joy = 0;
    switch(i){
        case 1: joy = GLFW_JOYSTICK_1; break;
        case 2: joy = GLFW_JOYSTICK_2; break;
        case 3: joy = GLFW_JOYSTICK_3; break;
        case 4: joy = GLFW_JOYSTICK_4; break;
    }
    
    int n = glfwGetJoystickParam(joy,GLFW_PRESENT);
    
    if(n == GL_FALSE){
        printf("Joystick[%i] not present\n",i);
        return;
    }
    
    int axes = glfwGetJoystickParam(joy,GLFW_AXES);
    int buttons = glfwGetJoystickParam(joy,GLFW_BUTTONS);
    
    printf("Joystick[%i]: axes[%i] buttons[%i]\n",i,axes,buttons);
}
*/

int main(int argc, char *argv[]){

	ros::init(argc, argv, "joystick");
	
	int joy_num = 0;
	float hz = 30.0;
	
	po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("num", po::value<int>(),"which joystick to use, default is 0")
        ("hz", po::value<float>(), "polling frequency in Hz, default is 30")
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
	
    if( !glfwInit() ){
        ROS_ERROR("Failed to initialize GLFW" );
        exit(1);
    }
    
    JoyStick js(joy_num);
    js.spin(hz);

    // Terminate GLFW
    glfwTerminate();

    // Exit program
    exit(0);
}

