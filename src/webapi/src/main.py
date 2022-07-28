#!/usr/bin/env python

import webapi
import rospy

if __name__ == '__main__':
    rospy.init_node("webapi")
    webapi = webapi.WebAPI()
    webapi.start()