#!/usr/bin/env python

import sys
# asher todo: make generic path
sys.path.append('/home/afst/Desktop/FormulaProjectUE18_4/Formula_Data/Data_Dev/')

import setup_path
import airsim

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import pp_msg 
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
publisherInput = { "airsimPose" : {
                                    "data_class" : PoseStamped , 
                                    "queue_size": 10,
                                    "handler" : publish_airsimPose,
                                    "handler_args" : sim
                                    },
                  "test" : {
                            "data_class" : String , 
                            "queue_size": 1,
                            "handler" : publish_test,
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
