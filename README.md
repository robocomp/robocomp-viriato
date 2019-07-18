# robocomp-viriato

# Manual

## Turning on/off the robot

### Turning on the platform: base buttons and led indicators

To turn on the platform the red and black buttons must be pressed simultaneously for about three seconds, until the red led turns on. That indicates that the platform is connected, but not its motors necessarily. To turn on the motors the blue button must be pressed until the blue leds is turned on (about a third of a second). Pressing the red button is enough to turn off the platform.

![base_controls](https://github.com/robocomp/robocomp-viriato/blob/master/files/manual/base_controls.jpg?raw=true "Base controlsd")

The leds on the right indicate the status of the robot in terms of charge.


### Setting up the operating system for the robot
 * Include the user that is going to run the robot's components is in the group *dialout*. Logout and log back in.
 * Make sure that the *udev* rules files have been installed (copy file *files/777-VIRIATO.rules* to *etc/udev/rules.d/*) and restart the *udev* service.
 * Run *files/setDevices1.sh* every time you plug/un-plug the Viriato's USB connectors to reset the config files. Bare in mind that although UDEV creates the /dev/viriatoctl and /dev/viriatonrg files, the component must use the actual device files. This scripts makes sure that the files are properly set up.
 * The motor controllers' API can be found in https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/nxtgen-downloads-1/application-programming-interface/8-linux-api/file

### Deploying components
