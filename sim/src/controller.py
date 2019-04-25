#!/usr/bin/env python

import sys
# asher todo: make generic path
sys.path.append('/home/afst/Desktop/FormulaProjectUE18_4/Formula_Data/Data_Dev/')

import setup_path
import airsim

import rospy
import tf

# imports for Control team
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D

from custom_msgs.msg import *



# imports for OrbSlam
from sensor_msgs.msg import CompressedImage



import time
from simulationClass.simulationClass import Simulation

from subscriber_callbacks import *
from publisher_handlers import *

from collections import defaultdict
#global moves
rospy.init_node('simControl', anonymous=True)
rate = rospy.Rate(10)  # 10hz

sim = Simulation()


#asher todo: extract to config file
publisherInput = { "carPoseForPurePursuit" : {
                                    "data_class" : Pose2D,
                                    "queue_size": 10,
                                    "handler" : publish_carPoseForPurePursuit,
                                    "handler_args" : sim
                                    },
                  "test" : {
                            "data_class" : String , 
                            "queue_size": 1,
                            "handler" : publish_test,
                            "handler_args" : sim
                            },
                  "camera/image_raw" : {
                            "data_class" : CompressedImage ,
                            "queue_size": 1,
                            "handler" : publish_RealAndSegImageData,
                            "handler_args" : sim
                            }
                  }

subscriberInput = { "pp_controller" : {"data_class" : pp_msg , 
                                       "callback": callback_pp_controller, 
                                       "callback_args" : sim}}



def setPublishers(publishArgs):
    publishers = {}
    if publishArgs is not None:
        for topic,args  in publishArgs.iteritems():
            publishers[topic] = {}
            publishers[topic]["object"] = rospy.Publisher(topic, args["data_class"], queue_size = args["queue_size"])
            publishers[topic]["handler"] = args["handler"]
    return publishers
    
def setSubscribers(subscribeArgs):
    subscribers = []
    if subscribeArgs is not None:
        for topic,args  in subscribeArgs.iteritems():
            subscribers.append(rospy.Subscriber(topic,data_class = args["data_class"], callback= args["callback"],
                                                callback_args = args["callback_args"]))
    return subscribers

def simControl():
    
    pubs = setPublishers(publisherInput)
    subs = setSubscribers(subscriberInput)
    
    while not rospy.is_shutdown():
        for pub in pubs.itervalues():
            pub["handler"](pub["object"], sim)
        
        rate.sleep()


if __name__ == '__main__':
    try:

        simControl()
    except rospy.ROSInterruptException:
        pass
