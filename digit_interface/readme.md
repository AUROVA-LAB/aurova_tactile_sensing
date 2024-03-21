In this package, there are two important files: digit.py in scripts folder, and digit.launch in launch folder.

Currently, the digit.py script is prepared to run two DIGIT sensors simultaneously. It is important to indicate the serial number of the DIGIT sensors in the digit.launch file. 

To use this package, you first need to create a catkin workspace, add the package, and compile it with catkin_make. Then, you need to change the serial numbers of the DIGITS to use the ones of your
sensors, and then you can execute the roslaunch command.
