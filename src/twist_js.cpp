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

//--- C++ ---------------------
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
        std::cout << desc << "\n";
        return 0;
    }
    
    if (vm.count("num")){
        joy_num = vm["num"].as<int>();
    }
    
    if (vm.count("hz")){
        hz = vm["hz"].as<float>();
    }
	
	// init the SDL2 library
    // Initialize SDL (Note: video is required to start event loop) 
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        ROS_ERROR("[-] Failed to initialize SDL2: %s", SDL_GetError());
        exit(1);
    }
    
    //-------------------------------------------------------------------------------
    
    TwistJoyStick js(joy_num);
    js.setUpPublisher();
    js.spin(hz);
    
    // Shut things down
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);

    // Exit program
    exit(0);
}

