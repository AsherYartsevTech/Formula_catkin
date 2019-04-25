#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import pp_msg
from geometry_msgs.msg import Pose2D

import rospy
from  cv2 import imread, imencode
from pyquaternion import Quaternion
import numpy as np

from PIL import Image
import numpy as np

def load_image(infilename) :
    img = Image.open(infilename)
    img.load()
    data = np.asarray( img, dtype="uint8" )
    return data

# imports for OrbSlam
from sensor_msgs.msg import CompressedImage

def publish_carPoseForPurePursuit(publisher, sim):
    car_state = sim.client.getCarState()
    pos = car_state.kinematics_estimated.position
    orientation = car_state.kinematics_estimated.orientation


    # populate PoseStamped ros message
    # simPose = pp_pos()
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
    image = load_image("/home/afst/Documents/AirSim/2019-04-07-20-55-13/images/testImage.png")

    imageMsg = CompressedImage()
    imageMsg.header.stamp = rospy.Time.now()
    imageMsg.format = "png"
    imageMsg.data = np.array(imencode('.png', image)[1]).tostring()
    # rospy.loginfo(imageMsg)
    publisher.publish(imageMsg)

def publish_test(publisher, sim):
    publisher.publish('MACABBI HAIFA')
