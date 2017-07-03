#!/usr/bin/env python

import sys
import rospy
from dbot_ros_msgs.srv import TrackObject

def find_and_track_object(object_name,auto_detect,auto_confirm):

  service = "/object_tracker_controller_service"

  rospy.wait_for_service(service)

  try:
    find_and_track = rospy.ServiceProxy(service, TrackObject)
  except Exception as e:
    print "failed to create proxy to "+service+": "+str(e)

  try :
    find_and_track(object_name,auto_detect,auto_confirm)
  except Exception as e:
    print "failed to call service "+service+": "+str(e)
    

def parse_args(args):

  if len(args)<3 : 
    raise Exception("usage: python object_tracker_controller_service_call <object name> <auto detect> <auto confirm>")

  object_name  = str(args[0])

  if args[1].lower()=="true": auto_detect=True
  elif args[1].lower()=="false": auto_detect=False
  else :
    raise Exception("auto_detect argument should be either true or false")

  if args[2].lower()=="true": auto_confirm=True
  elif args[2].lower()=="false": auto_confirm=False
  else :
    raise Exception("auto_confirm argument should be either true or false")

  return object_name,auto_detect,auto_confirm


def execute():

  rospy.init_node('object_tracker_controller_service', anonymous=True)  
  args = sys.argv[1:]
  object_name,auto_detect,auto_confirm = parse_args(args)
  find_and_track_object(object_name,auto_detect,auto_confirm)
  rospy.spin()


if __name__ == "__main__":

  execute()


