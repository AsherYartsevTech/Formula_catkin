#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
from pyquaternion import Quaternion

def publish_airsimPose(publisher, sim):
    
    car_state = sim.client.getCarState()
    pos = car_state.kinematics_estimated.position
    orientation = car_state.kinematics_estimated.orientation
    

    # populate PoseStamped ros message
    simPose = PoseStamped()
    simPose.pose.position.x = pos.x_val
    simPose.pose.position.y = pos.y_val
    simPose.pose.position.z = pos.z_val
    
    #asher todo: publish converterFromQToRad.radians
    converterFromQToRad = Quaternion(orientation.w_val,orientation.x_val,orientation.y_val,orientation.z_val)
    
    
    simPose.pose.orientation.w = orientation.w_val
    simPose.pose.orientation.x = orientation.x_val
    simPose.pose.orientation.y = orientation.y_val
    simPose.pose.orientation.z = orientation.z_val
    
    simPose.header.stamp = rospy.Time.now()
    simPose.header.seq = 1
    simPose.header.frame_id = "simFrame"
    
    # log PoseStamped message
    rospy.loginfo(simPose)
    #publish PoseStamped message
    publisher.publish(simPose)


def publish_test(publisher, sim):
    publisher.publish('MACBBI HAIFA')
