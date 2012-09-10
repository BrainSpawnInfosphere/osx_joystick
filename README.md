# ROS Package: OSX Joystick

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/osx_joystick

The standard Linux PS3 joystick drivers do not work on OSX. This package leverages 
[GLFW](http://www.glfw.org) to read joysticks under OS X. I currently have only tested 
it with the PS3 controller.

### Homebrew Dependencies

Follow the [instructions](http://ros.org/doc/api/rosdep2/html/contributing_rules.html) to
setup rosdep correctly. If you don't want to do that, then uncomment rosdep glfw in 
manifest.xml. Then just add the homebrew library as normal.

The required [Homebrew](http://mxcl.github.com/homebrew/) formula for OSX can be 
installed using:

    brew update
    brew install glfw
    
## Bluetooth PS3 Controller Setup

These instructions are a modification of those found at [here](http://thp.io/2010/psmove/)

1. Turn Bluetooth ON and press the PS button. A window will pop up asking for a PIN, 
ignore it and open the bluetooth preferences. Write down the Bluetooth address, 
mine is: e0-ae-5e-0e-1c-eb.
2. Turn Bluetooth OFF and copy the file /Library/Preferences/com.apple.Bluetooth.plist 
to somewhere else (e.g. your $HOME)
3. Open the plist with XCode and add your controller's address to the HIDDevices key 
(press the little plus). So I now have **Item 0** String with an address that was 
already there and a new **Item 1** String with my controller's address e0-ae-5e-0e-1c-eb 
(all lower case).
4. Now copy the file back to /Library/Preferences and make sure it's owned by root, group 
wheel (you can do this using chown root:wheel com.apple.Bluetooth.plist)
5. Switch on Bluetooth, and press the PS button on the controller - it should now pair 
without asking for a PIN

Now if you don't have XCode installed for some strange reason or need to do this all on
the command line, substitute these for the appropriate steps (**Note I have not checked
to see if these work**):

* Convert it to XML format using: plutil -convert xml1 com.apple.Bluetooth.plist
* Open the file with your favorite text editor like pico, and edit as above.
* Convert it back to binary Plist format using: plutil -convert binary1 
com.apple.Bluetooth.plist

## Command Line

	rosrun osx_joystick joystick --num --hz --help

 * help Prints the help message	
 * num The number of the joystick to connect too
 * hz The refresh rate to poll the joystick at (default: 30 Hz)
	
### Example:

    rosrun osx_joystick joystick --num 3 --hz 50

## Published Topics: 
**joy[0-15]:** 

The node publishes a standard [joystick message](http://www.ros.org/doc/api/sensor_msgs/html/msg/Joy.html)

## Buttons

![axes](http://i1268.photobucket.com/albums/jj568/mars_university/ps3_buttons.jpg)

![buttons](http://i1268.photobucket.com/albums/jj568/mars_university/ps3_buttons_front.jpg)

0. h
1. h
2. h
3. h

## Axes

![buttons](http://i1268.photobucket.com/albums/jj568/mars_university/ps3_axes.jpg)

## ToDo

 * document what buttons are what
 * document the axes
 * grab PS3 pictures from the linux package
 * see if the Wii works :) -- doesn't see to work
 * need to double check the license is compatible with BSD 


