#README for obstacle_remover -> dataFusion

#Add this line to the launch file

<node pkg = "tf" type = "static_transform_publisher" name = "camera_to_laser" args = " 0.19 -0.19 -0.05 0 0.2442 0 laser camera 100" />

tf publisher:
rosrun tf static_transform_publisher 0.19 -0.19 -0.05 0 0.2442 0 laser camera 100

#the 6 numbers represent the pose: x, y, z, roll , pitch, yaw