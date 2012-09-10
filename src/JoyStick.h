/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 9/9/2012
 *********************************************************************
 *
 * Change Log:
 *  9 Sep 2012 Created
 *
 **********************************************************************
 *
 *
 *
 */

#ifndef __JOY_STICK_H__
#define __JOY_STICK_H__

//--- C++ ---------------------
#include <math.h>
#include <stdlib.h> // ?
#include <stdio.h>  // ?
#include <string.h>

//--- GLFW --------------------
#include <GL/glfw.h>

//--- ROS ---------------------
#include <ros/ros.h>
#include <sensor_msgs/Joy.h> // joystick
#include <geometry_msgs/Twist.h> // twist to command robot

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
            ROS_ERROR("axes[%i] buttons[%i]",num_axes,num_buttons);
            ROS_ERROR("Couldn't connect to joystick[%i]",joy_num);
            exit(1);
        }
        
        ROS_INFO("==================================");
        ROS_INFO("Joystick%i: axes[%i] buttons[%i]",joy_num,num_axes,num_buttons);
        ROS_INFO("==================================");
        
    }
    
    virtual void setUpPublisher(void){
        //ros::NodeHandle n("~");
        ros::NodeHandle n;
        char joy_name[32];
        sprintf(joy_name, "joy%d",joy_num);
        joy_pub = n.advertise<sensor_msgs::Joy>(joy_name, 50);
        
        
        //ROS_INFO("joystick");
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
    
    // main loop
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
                    ROS_ERROR("Can't connect to joystick[%i] ... exiting",joy_num);
                    return;
                }
            }
            
            publishMessage();

            ros::spinOnce();
            r.sleep();
        }
	}
	
	virtual void publishMessage(void){
        
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
    }
	
protected:
    int joy_num;    // what joystick
    int num_axes;   // how many axes
    int num_buttons; // how many buttons
    float a[32];    // tmp storage
    unsigned char b[32]; // tmp storage
    
    //sensor_msgs::Joy joy_msg; // should i keep this?
	ros::Publisher joy_pub;
	ros::Publisher twist_pub;
};


/**
 * Modified JoyStick class that publishes a Twist message instead of a Joy message. This
 * makes it easier to drive a robot.
 */
class TwistJoyStick : public JoyStick {
public:

    TwistJoyStick(int i=GLFW_JOYSTICK_1) : JoyStick(i){
        ;
    }
    
    virtual void setUpPublisher(void){
        //ros::NodeHandle n("~");
        ros::NodeHandle n;
        char joy_name[32];
        sprintf(joy_name, "cmd%d",joy_num);
        twist_pub = n.advertise<geometry_msgs::Twist>(joy_name, 50);
        
        //ROS_INFO("twist");
    }
    
    virtual void publishMessage(void){
        
        // publish joystick message
        geometry_msgs::Twist msg;
        
        msg.linear.x = a[1];
        msg.linear.y = a[0];
        msg.linear.z = 0.0;
        
        msg.angular.x = 0.0;
        msg.angular.y = a[3]; // pitch
        msg.angular.z = a[2]; // yaw
        
        twist_pub.publish(msg);
    }

};

#endif

