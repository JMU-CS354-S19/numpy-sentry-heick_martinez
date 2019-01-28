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
        self.intrusion = False

        while not rospy.is_shutdown():
            if self.intrusion:
                self.sound.value = 1
                self.pub.publish(self.sound)

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """
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
 
            if (norm/self.average) > 1.6:
                self.intrusion = True
                rospy.loginfo("intrusion")
                rospy.loginfo(intrude)
            else:
                self.intrusion = False

            
        # YOUR CODE HERE.
        # HELPER METHODS ARE GOOD.

        
if __name__ == "__main__":
    node = SentryNode()
