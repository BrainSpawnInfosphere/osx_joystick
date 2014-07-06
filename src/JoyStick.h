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
 *  5 Jul 2014 Changed to SDL to support PS4 controller
 *
 **********************************************************************
 *
 *
 *
 */

#ifndef __JOY_STICK_H__
#define __JOY_STICK_H__

//--- C++ ---------------------
#include <string.h>

//--- SDL2 --------------------
#include <SDL.h>

//--- ROS ---------------------
#include <ros/ros.h>
#include <sensor_msgs/Joy.h> // joystick
#include <geometry_msgs/Twist.h> // twist to command robot

/**
 * Simple joystick driver using SDL. It is primarily geared towards the PS4 controller.
 */
class JoyStick {
public:
    JoyStick(int i=0){
        joy_num = i;
        joystick = SDL_JoystickOpen(joy_num);
        
        if(joystick == NULL){
            ROS_ERROR("Couldn't connect to joystick[%i]",joy_num);
            exit(1);
        }
        
        num_axes = SDL_JoystickNumAxes(joystick);
        num_buttons = SDL_JoystickNumButtons(joystick);
        
        if(num_axes == 0 || num_buttons == 0){
            ROS_ERROR("axes[%i] buttons[%i]",num_axes,num_buttons);
            ROS_ERROR("Couldn't connect to joystick[%i]",joy_num);
            exit(1);
        }
        
        ROS_INFO("==================================");
        ROS_INFO("Joystick has %d axes, %d hats, %d balls, and %d buttons",
           SDL_JoystickNumAxes(joystick), SDL_JoystickNumHats(joystick),
           SDL_JoystickNumBalls(joystick), SDL_JoystickNumButtons(joystick));
        ROS_INFO("==================================");
        
        // resize message fields here once
        joy_msg.axes.resize(num_axes);
        joy_msg.buttons.resize(num_buttons+1); // hat is last
        
        SDL_JoystickClose(joystick);
        
    }
    
    virtual void setUpPublisher(void){
        //ros::NodeHandle n("~");
        ros::NodeHandle n;
        char joy_name[32];
        sprintf(joy_name, "joy%d",joy_num);
        joy_pub = n.advertise<sensor_msgs::Joy>(joy_name, 50);
    }
    
    // main loop
    void spin(float hz){
        ros::Rate r(hz);
        unsigned int err_cnt = 0;

        // Main Loop -- go until ^C terminates
        while (ros::ok()){
            
            get();
            publishMessage();

            ros::spinOnce();
            r.sleep();
        }
	}
	
	void mapAxesAndButtons(){
		;
	}
	
	virtual void get(void){
	
        joystick = SDL_JoystickOpen(joy_num);
        
        // publish joystick message
        joy_msg.header.stamp = ros::Time::now();
        
        // copy axes
        for(int i=0;i<3;++i) joy_msg.axes[i] = SDL_JoystickGetAxis(joystick,i);
        
        // PS4 controller correction
        // for some strange reason, the right js axis isn't with the others, so 
        // I will move them around so left js is 0,1 and right js is 2,3
        joy_msg.axes[3] = SDL_JoystickGetAxis(joystick,5); // right js y-axis
        joy_msg.axes[4] = SDL_JoystickGetAxis(joystick,3); // left trigger
        joy_msg.axes[5] = SDL_JoystickGetAxis(joystick,4); // right trigger
        
        // copy buttons
        for(int i=0;i<num_buttons;++i) joy_msg.buttons[i] = SDL_JoystickGetButton(joystick,i);
        
        // ROS doesn't support hat buttons (per say) in their messages, so I am 
        // adding an extra button to account for it
        joy_msg.buttons[14] = SDL_JoystickGetHat(joystick,0);
        
        
        SDL_JoystickClose(joystick);
	}
	
	virtual void publishMessage(void){
        
        joy_pub.publish(joy_msg);
    }
	
protected:
    int joy_num;    // what joystick
    int num_axes;   // how many axes
    int num_buttons; // how many buttons
    SDL_Joystick *joystick;
    
    sensor_msgs::Joy joy_msg; 
	ros::Publisher joy_pub;
};


/**
 * Modified JoyStick class that publishes a Twist message instead of a Joy message. This
 * makes it easier to drive a robot.
 */
class TwistJoyStick : public JoyStick {
public:

    TwistJoyStick(int i=0) : JoyStick(i){
        ;
    }
    
    virtual void setUpPublisher(void){
        ros::NodeHandle n;
        char joy_name[32];
        sprintf(joy_name, "joy%d",joy_num);
        twist_pub = n.advertise<geometry_msgs::Twist>(joy_name, 50);
        
        //ROS_INFO("twist");
    }
    
    virtual void publishMessage(void){
        
        // publish joystick message
        geometry_msgs::Twist msg;
        
        msg.linear.x = joy_msg.axes[1];
        msg.linear.y = joy_msg.axes[0];
        msg.linear.z = 0.0;
        
        msg.angular.x = 0.0;
        msg.angular.y = joy_msg.axes[3]; // pitch
        msg.angular.z = joy_msg.axes[2]; // yaw
        
        twist_pub.publish(msg);
    }
    
protected:
	ros::Publisher twist_pub;
};

#endif

