#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Author: 
Version:
"""

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kobuki_msgs.msg import Sound
import numpy as np


class SentryNode(object):
    """Monitor a vertical scan through the depth map and create an
    audible signal if the change exceeds a threshold.

    Subscribes:
         /camera/depth_registered/image
       
    Publishes:
        /mobile_base/commands/sound

    """

    def __init__(self):
        """ Set up the Sentry node. """
        rospy.init_node('sentry')
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
        
        self.pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1) 
        self.sound = Sound()
        self.p = np.array([])
        self.first = True
        self.average = 1
 

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """
        rospy.loginfo("Depth")
        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        if self.first:
            self.p = depth[:, 240]
            self.first = False
            
        else:
            p2 = self.p #[~np.isnan(self.p)]
            c = depth[:, 240]
            #c = c[~np.isnan(c)]
            norm = np.absolute(np.subtract(c, p2))
            norm = norm[~np.isnan(norm)]
            norm = np.sum(norm)
            self.average = self.average * .5 + norm * (1 - .5)
            intrude = norm/self.average
            rospy.loginfo(intrude)
            if (norm/self.average) > 1.4:
                self.sound.value = 0
                rospy.loginfo("intrusion")
            else:
                self.sound.value = 1
                rospy.loginfo("No intrustion")

            
        # YOUR CODE HERE.
        # HELPER METHODS ARE GOOD.

    def start(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.pub.publish(self.sound)
            rospy.loginfo("publish")
            rate.sleep()
        
        
if __name__ == "__main__":
    node = SentryNode()
    rospy.loginfo("starting")
    node.start()
