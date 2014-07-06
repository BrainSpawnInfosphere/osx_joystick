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
 * Author: Kevin J. Walchko on 6/6/2014
 *********************************************************************
 *
 * Change Log:
 *  6 Jul 2014 Created
 *
 **********************************************************************
 *
 * Simple ncurses display of the PS4 controller.
 *
 */

#define NODE_VERSION 3.0

#include <ros/ros.h>
#include <sensor_msgs/Joy.h> // joystick

#include <ncurses.h>
#include <stdio.h>
#include <stdlib.h>

static sensor_msgs::Joy js;

static WINDOW *screen;


class JoyStickWindow {
public:
	JoyStickWindow(ros::NodeHandle &node);
	void loop();

protected:
	int drawScreen(void);
	void screen_init(void);
	void msgUpdate(const sensor_msgs::Joy::ConstPtr& joy_msg);
	
	WINDOW *screen;
	sensor_msgs::Joy js;
	ros::NodeHandle n;
	ros::Subscriber joystick;
};
	
JoyStickWindow::JoyStickWindow(ros::NodeHandle &node){
	n = node;
	joystick = n.subscribe<sensor_msgs::Joy>("/joy0", 1, &JoyStickWindow::msgUpdate, this);
	
	// Need to resize the axes and button arrays
	js.axes.resize(6);
	js.buttons.resize(15); // hat is 15th
	
	screen_init();
	
	if(screen == NULL){
		delwin(screen);
		endwin();
		ROS_ERROR("[-] Couldn't setup ncurses");
		exit(1);
	}
}

inline char bm(int i){ return (i > 0 ? 'X' : ' ');}

// stole this from SDL so I didn't have to link to it
#define 	SDL_HAT_CENTERED   0x00
#define 	SDL_HAT_UP   0x01
#define 	SDL_HAT_RIGHT   0x02
#define 	SDL_HAT_DOWN   0x04
#define 	SDL_HAT_LEFT   0x08
#define 	SDL_HAT_RIGHTUP   (SDL_HAT_RIGHT|SDL_HAT_UP)
#define 	SDL_HAT_RIGHTDOWN   (SDL_HAT_RIGHT|SDL_HAT_DOWN)
#define 	SDL_HAT_LEFTUP   (SDL_HAT_LEFT|SDL_HAT_UP)
#define 	SDL_HAT_LEFTDOWN   (SDL_HAT_LEFT|SDL_HAT_DOWN)

inline const char* hat(int i){
	if(i == SDL_HAT_CENTERED) return "centered";
	else if(i == SDL_HAT_UP) return "up";
	else if(i == SDL_HAT_RIGHT) return "right";
	else if(i == SDL_HAT_DOWN) return "down";
	else if(i == SDL_HAT_LEFT) return "left";
	else if(i == SDL_HAT_RIGHTUP) return "right-up";
	else if(i == SDL_HAT_RIGHTDOWN) return "right-down";
	else if(i == SDL_HAT_LEFTUP) return "left-up";
	else if(i == SDL_HAT_LEFTDOWN) return "left-down";
	
	return "error";
}

/**
 * Notes:
 
 Triggers are at -32768 unpressed and 32767 pressed
 
 Axes with values between -32768 to 32767: 
 +----> y
 |
 |
 v
 x
 
 Axes:
 0 L x
 1 L y
 2 R x
 3 R y
 4 L tr
 5 R tr
 
 Buttons:
 0 square
 1 x
 2 circle
 3 triangle
 4 L1
 5 R1
 6 L2
 7 R2
 8 share
 9 options
 10 L js
 11 R js
 12 PS button
 13 Pad
 14 hat
 */
int JoyStickWindow::drawScreen(void){
	
	int row = 0;
	
	wclear(screen); // clear screen of all contents
	
	curs_set(row++);
	mvwprintw(screen,row++,1,"+------------------------------------------+");
	mvwprintw(screen,row++,1,"|       PS4 Joystick %.2f        ", NODE_VERSION);
	mvwprintw(screen,row++,1,"|       Msg Sequence: %d        ",js.header.seq);
	mvwprintw(screen,row++,1,"+------------------------------------------+");
	
	row++;
	
	mvwprintw(screen,row++,6,"Left Stick: %5.1f %5.1f \t Right Stick: %5.1f %5.1f",
				 js.axes[0],js.axes[1],js.axes[2],js.axes[3]);
	mvwprintw(screen,row++,6,"Left Trigger: %5.1f \t Right Trigger: %5.1f",
				 js.axes[4],js.axes[5]);
	
	row++;
	
	mvwprintw(screen,row++,6,"Shoulder/Trigger Buttons: ");
	mvwprintw(screen,row++,2*6,"L1:%c  L2:%c  R1:%c  R2:%c ",
		bm(js.buttons[4]),bm(js.buttons[6]),bm(js.buttons[5]),bm(js.buttons[7]));
		
	row++;
	
	mvwprintw(screen,row++,6,"JoyStick Buttons: ");
	mvwprintw(screen,row++,2*6,"Left JS:%c  Right JS:%c ",
		bm(js.buttons[10]),bm(js.buttons[11]));
	
	row++;
	
	mvwprintw(screen,row++,6,"Symbol Buttons: ");
	mvwprintw(screen,row++,2*6,"square:%c  x:%c  circle:%c  triangle:%c ",
		bm(js.buttons[0]),bm(js.buttons[1]),bm(js.buttons[2]),bm(js.buttons[3]));
	
	row++;
	
	mvwprintw(screen,row++,6,"Other Buttons: ");
	mvwprintw(screen,row++,2*6,"Share:%c  Options:%c  PS:%c  Pad:%c ",
		bm(js.buttons[8]),bm(js.buttons[9]),bm(js.buttons[12]),bm(js.buttons[13]));
	
	row++;
	
	mvwprintw(screen,row++,6,"Hat: %s ", hat(js.buttons[14]));
	
	row++;
	
	mvwprintw(screen,row++,6,"Raw Buttons (plus hat): ");
	for(int i=0; i<7; ++i) mvwprintw(screen,row,6+i*2,"%d ", js.buttons[i]);
	row++;
	for(int i=7; i<15; ++i) mvwprintw(screen,row,6+(i-7)*2,"%d ", js.buttons[i]);
	row++;
	
	mvwprintw(screen,row++,1,"------- Menu ------");
	
	mvwprintw(screen,row++,6,"[q]uit");
	
	box(screen,0,0); // draw box around screen
	
	wrefresh(screen); // display text written to screen
	refresh();
	
	return 0;
}

void JoyStickWindow::screen_init(void) {
   screen = initscr(); // initialize ncurses
   noecho(); // don't echo characters to screen from keyboard input
   cbreak(); // get user input after every character
   nodelay(screen, TRUE);
   wclear(stdscr);
   wclear(screen); // clear screen of all contents
   wrefresh(screen);  // update window
   curs_set(FALSE);
}

void JoyStickWindow::msgUpdate(const sensor_msgs::Joy::ConstPtr& joy_msg){
	js = *joy_msg;
}

void JoyStickWindow::loop(){
	ros::Rate r(5.0);
	while(n.ok())
	{	
		char ans = 0;
		
		drawScreen();
		
		switch(ans = getch()){
			case 'q': // forward
				delwin(screen);
				endwin();
				return;
		}
		
		ros::spinOnce();
		r.sleep();
	}
	
	delwin(screen);
	endwin();
}

///////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv){	
	ros::init(argc, argv, "joystick_window");
	
	ros::NodeHandle n;
	
	JoyStickWindow joy(n);
	joy.loop();
	
	return 0;
}

// EOF
