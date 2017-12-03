Computer vision 

nodes structure:

usb_cam/camdriver (camdriver_node)-->obstacle_remover-->top_view-->CV_algo-->imgtolaserscan

Other Details:

bird_eye node calculates camera top-view transformation matrix and saves it in top_view.txt

add head_camera.yaml (caliberated for Genius webcam) file to ~/.ros/camera_info/ for camera caliberation

map_server node to covert to occupancy grid (not in use currently)

Launch files for vision : CV.launch, CV_algo.launch(to be run seperately)

Videostab node to stabalise the video feed from camera.