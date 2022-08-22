#!/usr/bin/env python

from CommUDP import CommUDP
import rospy

if __name__ == '__main__':
    rospy.init_node("comm_udp_main")
    comm_udp = CommUDP()
    # comm_udp.start()