#sometimes you need to install actual programs:
sudo apt install python3-pip
sudo apt install cmake
sudo apt install curl

#sometimes, python needs a module. Basically, if it says "module not found" then do "sudo pip3 install <module>"
sudo pip3 install serial
sudo pip3 install zmq
sudo pip3 install pyproj==1.9.6
sudo pip3 install keyboard
sudo pip3 install matplotlib
sudo pip3 install cython
sudo pip3 install numpy


#gazebo self installs with:
curl -sSL http://get.gazebosim.org | sh



#the gazebo simulator folder needs to be rebuilt for your computer:
#from the electronic differential directory
cd gazebo-simulator/build
rm CMakeCache.txt
cmake ../
make
cp libvelodyne_plugin.so ../../simulator-framework/
mkdir ~/.gazebo/models
cp -rf my_robot ~/.gazebo/models

#at this point you should be able to run the script
python3 executeSim.py

#keep an eye out at the start before the wall of number shows up for errors.

