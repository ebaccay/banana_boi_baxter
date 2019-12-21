#!/usr/bin/env
# Module Imports
import rospy
import cv2, cv_bridge
import os, sys
# Scientific Computing Imports
import numpy as np
import numpy.ma as ma
from numpy import linalg
from sklearn.decomposition import PCA
# ROS Topic Imports
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

# Hardcoded Variable Definitions
img_capture_pos = [0.820, -0.070, -0.021]
knife_wait_pos = [0.050, 0.850, 0.068]
img_wait_pos = [0, 0, 0] # TODO: Change this to correct coordinates

default_gripper_orientation = [0.0, 1.0, 0.0, 0.0]
table_height_robot_space = -0.362

knife_arm = 'left'
camera_arm = 'right'
img_capture_camera = '/cameras/left_hand_camera/image'
img_pixel_width = 50  #FIXME: Get the right number
img_pixel_height = 50 # FIXME: Get the right number

table_width = 50 # FIXME: Get the right number in METERS
table_height = 50 # FIXME: Get the right number in METERS

pixel_mapping = get_pixel_mapping()


def main():
	move_to(knife_start_pos.extend(default_gripper_orientation), knife_arm)
	move_to(img_capture_pos.extend(default_gripper_orientation), camera_arm)
	clustered_img, normal_img = img_capture(img_capture_camera)


	#Clustered img returns an int ndarray

	#Mask array and only select the 1's from the cluster using mask to run PCA on
	#Wait actually coordinate array wont match shape cause its got tuples while clustered_img does not so gotta do something  about that
	masked_img = ma.masked_where(clustered_img == 2,  coordinate_array)
	#Time to run the PCA guy


	#First component gives banana axis and we want to pick orthogonal to that so that should just be the second component
	components = calc_axis(masked_img)
	pick_along = components[1]


	#TODO convert the component to the right rotation in terms of baxter wrist

	#TODO calculate center of banana from pixels


	banana_pos = [x_banana, y_banana, z, 0, 1, yaw, 0]

	move_to(banana_pos, "right")

	#TODO gripper shit its on the computer gotta find it but this will close up grippers

	#Move banana up to not drag across table

	banana_pos[2] = -0.021

	#AR tag location or hardcode location
	final_des = [x_final, y_final, z, 0, 1, 0, 0]

	move_to(final_des, "right")

	#TODO Open Gripper

	#Move away right arm just play around with this and hardcode it like the starting position

	right_arm_final = [1, 1, 1, 0, 1, 0, 0]
	move_to(right_arm_final, "right")

	#Time to do some Cutting

	run_cut_routine()



def img_capture(topic):
	bridge = cv_bridge.CvBridge()

	this_file = os.path.dirname(os.path.abspath(__file__))
	IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'

	def image_callback(ros_img):
		cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
		cv2.imwrite(IMG_DIR + "/cv_image.jpg", cv_image)
		cv2.imshow('Image', cv_image)
		cv2.waitKey(1)

	rospy.init_node('Camera_Subscriber',anonymous=True)
	rospy.Subscriber(topic, Image, image_callback)

	rospy.spin()
	cv2.destroyAllWindows()

	return image_processing.segment()


def get_pixel_mapping():
	pixel_x_differential = table_width / img_pixel_width
	pixel_y_differential = table_height / img_pixel_height

	pixel_mapping = {}

	left_curr = img_capture_pos[0] - (float(table_width) / 2)
	for x in img_pixel_width:
		bottom_curr = img_capture_pos[1] - (float(table_height) / 2)
		for y in img_pixel_height:
			pixel_mapping[(x, y)] = [left_curr, bottom_curr, table_height_robot_space]
			bottom_curr += pixel_y_differential
		left_curr += pixel_x_differential

	return pixel_mapping










def calc_axis(img):
	pca = PCA(n_components = 2)
	pca.fit(img)
	return pca_components_


def move_to(coordinates, move_arm):
    #CODE MODELED OFF IK EXAMPLE IN LAB 5
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = move_arm
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        #Alan does not have a gripper so replace link with 'right_wrist' instead
        link = arm + "_gripper"
        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = coordinates[0]
        request.ik_request.pose_stamped.pose.position.y = coordinates[1]
        request.ik_request.pose_stamped.pose.position.z = coordinates[2]
        request.ik_request.pose_stamped.pose.orientation.x = coordinates[3]
        request.ik_request.pose_stamped.pose.orientation.y = coordinates[4]
        request.ik_request.pose_stamped.pose.orientation.z = coordinates[5]
        request.ik_request.pose_stamped.pose.orientation.w = coordinates[6]

        try:
            #Send the request to the service
            response = compute_ik(request)

            #Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


def run_cut_routine(start_coords, end_coords, num_cuts):
	start_x, start_y, start_z = start_coords[0], start_coords[1], start_coords[2]
	end_x, end_y, end_z = end_coords[0], end_coords[1], end_coords[2]
	delta_x = float(end_x - start_x) / num_cuts
	delta_y = float(end_y - start_y) / num_cuts
	delta_z = float(end_z - start_z) / num_cuts




        #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = move_arm
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        # raw_input('Press [ Enter ]: ')
        start_cut_postion = [start_x, start_y, start_z, 0, 1, 0, 0]
        move_to(start_cut_postion, "left")

        while no_cuts:
            #IDK which directions the axis is so it could be plus or minus. I'm just using plus for now
            #Finish This
            new_x = start_x
            new_y = start_y
            new_z = z  #this is cutting motion

            temp_pos = [new_x, new_y, new_z, 0, 1, 0, 0]
            move_to(temp_pos, "left")

            #Bring Knife UP
            temp_pos[2] = new_z + delta_z
            move_to(temp_pos " left")


if __name__ == "__main__":
	main()
