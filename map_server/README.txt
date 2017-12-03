mapserver package

dependencies:
sudo apt-get install libsdl2-2.0-0 
sudo apt-get install libsdl2-image-dev
sudo apt-get install libsdl2-image-2.0-0
sudo apt-get install libsdl1.2-dbg
sudo apt-get install libsdl-image1.2-dev

command:
rosrun map_server map_server map.yaml

subscribes to /lane_image and publishes on /map (occupancy).
debug: use rviz to visualise.

