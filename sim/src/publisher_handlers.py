#!/usr/bin/env python

from geometry_msgs.msg import Pose2D

import rospy
import airsim
from  cv2 import imread, imencode
from pyquaternion import Quaternion

import numpy as np

def load_image(img) :
    data = np.asarray(img, dtype="uint8")
    return data


# imports for OrbSlam
from sensor_msgs.msg import CompressedImage

def publish_carPoseForPurePursuit(publisher, sim):
    car_state = sim.client.getCarState()
    pos = car_state.kinematics_estimated.position
    orientation = car_state.kinematics_estimated.orientation

    simPose = Pose2D()
    simPose.x = pos.x_val
    simPose.y = pos.y_val
    converterFromQToRad = Quaternion(orientation.w_val,orientation.x_val,orientation.y_val,orientation.z_val)
    simPose.theta = converterFromQToRad.radians

    # log PoseStamped message
    rospy.loginfo(simPose)

    #publish PoseStamped message
    publisher.publish(simPose)


def publish_RealAndSegImageData(publisher, sim):
    raw_image_list = sim.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    response = raw_image_list[0]
    print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)  # get numpy array
    img_rgba = img1d.reshape(response.height, response.width, 4)  # reshape array to 4 channel image array H X W X 4

    img_rgba = np.flipud(img_rgba)  # original image is flipped vertically

    imageMsg = CompressedImage()
    imageMsg.header.stamp = rospy.Time.now()
    imageMsg.format = "png"
    imageMsg.data = np.array(imencode('.png', img_rgba)[1]).tostring()
    # rospy.loginfo(imageMsg)
    publisher.publish(imageMsg)

def publish_test(publisher, sim):
    publisher.publish('MACABBI HAIFA')