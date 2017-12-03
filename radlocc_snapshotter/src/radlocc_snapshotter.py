#!/usr/bin/python
"""
Takes laser scan and camera snapshots for use with Matlab RADLOCC Laser-Camera Calibration Toolbox
Intended to be used at the same time as running the ROS camera calibration tool, 
just for visualization of likely checkboard recognition success, for instance:
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/global_shutter_mono_camera_throttled/image_raw
"""

import rospy
from sensor_msgs.msg import LaserScan, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class RADLOCCSnapshotter(object):
    """Class for saving camera images and laser scan data for camera-laser calibration"""

    def __init__(self):

        #Get the rosparams
        self.scan_topic = rospy.get_param("~scan_topic", '/scan')
        self.camera_topic = rospy.get_param("~camera_topic", \
                                            '/image')
        self.output_directory = rospy.get_param("~output_directory", '/home/swati/catkin_ws/Camera_Lidar_Calibration')
        self.range_cutoff = rospy.get_param("~range_cutoff", 5.5)

        #CvBridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        #laser data and camera timestamp files, as requested by RADLOCC toolbox
        self.image_number = 1
        laser_filename = self.output_directory + "/matlabLaserHorizontalData.txt"
        self.laser_file = open(laser_filename, 'w')
        camera_timestamp_filename = self.output_directory + "/VideoLogAsciiCalibration.txt"
        self.camera_timestamp_file = open(camera_timestamp_filename, 'w')

    def run(self):
        """Loop, waiting for the user to press enter to take laser+camera snapshots"""

        while not rospy.is_shutdown():
            print "Press enter to grab a camera-laser pair, or q to quit"
            input_str = raw_input()
            if "q" in input_str:
                break
            try:
                laser_msg = rospy.wait_for_message(self.scan_topic, LaserScan, timeout=0.25)
            except rospy.exceptions.ROSException as err:
                print "Failed to receive laser message: " + str(err)
                continue
            try:
                camera_msg = rospy.wait_for_message(self.camera_topic, Image, timeout=0.25)
            except rospy.exceptions.ROSException as err:
                print "Failed to receive camera message: " + str(err)
                continue

            #generate the laser scan string to write to file
            laser_str = self.process_laser_scan(laser_msg)

            #write the image to bmp file
            try:
                self.process_camera_image(camera_msg)
            except CvBridgeError:
                print "Failed to process camera image, tossing data"
                continue

            #only add the laser string to the file if the image wrote successfully
            self.image_number += 1
            self.laser_file.write(laser_str)


    def process_laser_scan(self, msg):
        """
        Generate the string to put into the matlabLaserHorizontalData file: 
        <timestamp> StartAngleRads AngleIncrementRads EndAngleRads RangeUnitType NoAngles [Ranges]
        """
        laser_str = str(msg.header.stamp.to_sec())
        laser_str += " %0.3f" % msg.angle_min + " %0.3f" % msg.angle_increment + \
                     " %0.3f" % msg.angle_max + " 3 " + str(len(msg.ranges))
        for range_val in msg.ranges:
            if range_val > self.range_cutoff:
                laser_str += " nan"
            else:
                laser_str += " %0.3f" % range_val
        laser_str += "\n"
        return laser_str


    def process_camera_image(self, msg):
        """Writes the current image to image#.bmp"""

        filename = self.output_directory + "/image%03d.bmp" % self.image_number
        print filename
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err: 
            print str(err)
            raise err
        try:
            cv2.imwrite(filename, cv_image)  
        except Exception as err:
            print str(err)
            raise err
        self.camera_timestamp_file.write(str(msg.header.stamp.to_sec()) + "\n")


if __name__ == '__main__':

    node_name = "radlocc_snapshotter"
    rospy.init_node(node_name)

    snapshot = RADLOCCSnapshotter()

    snapshot.run()

