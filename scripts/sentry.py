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
        
        rospy.Publisher('/mobile_base/commands/sound', Sound) \
        self.sound = Sound()
        self.p = np.array()
        self.first = true
        self.average = 1
        rospy.spin()

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """
        
        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        if first:
            p = depth[:, 240]
            
        else:
            c = depth[:, 240]
            norm = np.absolute(np.subtract(c - p))
            norm = np.sum(norm)
            average = average * .5 + norm * (1 - .5)
            
            if (norm/average) > 1.5:
                self.sound.value = 0
            else:
                self.sound.value = 1
            rospy.publish(self.sound)
            
        # YOUR CODE HERE.
        # HELPER METHODS ARE GOOD.


if __name__ == "__main__":
    SentryNode()
