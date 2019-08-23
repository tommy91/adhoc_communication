#!/bin/bash

clear 
cd ../..	# Moving on the catkin workspace directory
catkin_make
source devel/setup.bash
echo "Root password is needed to set SUID bit of the binary file to use raw socket connections.."
sudo chown root ./devel/lib/adhoc_communication/adhoc_communication
sudo chmod +s ./devel/lib/adhoc_communication/adhoc_communication
cd -  
