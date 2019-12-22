#!/usr/bin/env

################################################################################
#                               SECTION ZERO (0)                               #
################################################################################

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

################################################################################
#                               SECTION ONE (1)                                #
################################################################################

# Hardcoded Variable Definitions
img_capture_pos = [0.820, -0.070, -0.021]
knife_wait_pos = [0.050, 0.850, 0.068]
gripper_wait_pos = [0, 0, 0]
final_knife_cut_pos = [0, 0, 0]
img_wait_pos = [0, 0, 0]

default_gripper_orientation = [0.0, 1.0, 0.0, 0.0]
table_height_robot_space = -0.362
table_preheight_robot_space = table_height_robot_space + 0.152

knife_arm = 'left'
camera_arm = 'right'
gripper_arm = 'right'
img_capture_camera = '/cameras/left_hand_camera/image'
img_pixel_width = 320
img_pixel_height = 200

table_width = 0.914
table_height = 0.609

pixel_mapping = get_pixel_mapping()

################################################################################
#                               SECTION TWO (2)                               #
################################################################################

def main():
	# <--------- SUBSECTION A ----------> #
	# Move to reset postion
	move_to(knife_start_pos, default_gripper_orientation, knife_arm)
	move_to(img_capture_pos, default_gripper_orientation, camera_arm)
	clustered_img, normal_img = img_capture(img_capture_camera)  # Returns int ndarray
	points = grab_relevant_points(clustered_img)
	quaternion_axis = calc_axis(masked_img)  # PCA Calculation to acquire correct axis
	banana_centroid = pixel_mapping(tuple(centroids(points)))  # Centroid calculation to acquire banana position

	# <--------- SUBSECTION B ----------> #
	banana_pre_pos = banana_centroid + [table_preheight_robot_space]
	banana_final_pos = banana_centroid + [table_height_robot_space]
	move_to(banana_pre_pos, quaternion_axis, gripper_arm)
	move_to(banana_final_pos, quaternion_axis, gripper_arm)

	move_to(banana_pre_pos, quaternion_axis, gripper_arm)
	move_to(final_knife_cut_pos, quaternion_axis, gripper_arm)
	move_to(gripper_wait_pos, default_gripper_orientation, gripper_arm)
	move_to(final_knife_cut_pos, quaternion_axis, knife_arm)

	# <--------- SUBSECTION C ----------> #
	# Time to do some Cutting
	all_x_points = [p[0] for p in points]
	all_y_points = [p[1] for p in points]
	start_pos = [min(all_x_points), min(all_y_points)]
	end_pos = [max(all_x_points), max(all_y_points)]

	run_cut_routine(start_pos, end_pos, 4, quaternion_axis)

################################################################################
#                              SECTION THREE (3)                               #
################################################################################

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
			pixel_mapping[(x, y)] = [left_curr, bottom_curr]
			bottom_curr += pixel_y_differential
		left_curr += pixel_x_differential

	return pixel_mapping


def calc_axis(img):
	pca = PCA(n_components = 2)
	pca.fit(img)
	first_comp = pca_components_[1]
	return to_quaternion(-np.pi, 0, np.arctan(firstcomp[0] / firstcomp[1]))


def to_quaternion(yaw, pitch, roll):
    return [
		np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2),
		np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2),
		np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2),
		np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	]


def centroids(points):
	num_points = len(points)
	x_sum = sum([p[0] for p in points])
	y_sum = sum([p[1] for p in points])
	return [x_sum / num_points, y_sum / num_points]


def grab_relevant_points(xy_points):
	relevant_points = []
	for x in range(len(xy_points)):
		for y in len(xy_points[0]):
			if xy_points[x][y] > 0.5:
				relevant_points.append([x, y])
	return relevant_points


def move_to(coordinates, quaternion, move_arm):
	complete_position = coordinates.copy()
	complete_position.extend(quaternion)
	coordinates = complete_position

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


def run_cut_routine(start_coords, end_coords, num_cuts, quaternion_axis):
	start_x, start_y = start_coords[0], start_coords[1]
	end_x, end_y = end_coords[0], end_coords[1]
	delta_x = float(end_x - start_x) / num_cuts
	delta_y = float(end_y - start_y) / num_cuts

    while curr_x < end_x and curr_y < end_y:
	    start_cut_postion = [curr_x, curr_y, table_preheight_robot_space]
		end_cut_position = [curr_x, curr_y, table_height_robot_space]

	    move_to(start_cut_postion, quaternion_axis, knife_arm)
	    move_to(end_cut_postion, quaternion_axis, knife_arm)
		move_to(start_cut_postion, quaternion_axis, knife_arm)
		curr_x, cuyy_y = curr_x + delta_x, curr_y + delta_y


if __name__ == "__main__":
	main()
