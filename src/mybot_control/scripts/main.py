#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math
from walker import walker
from walker2 import walker2
from rflearning import train
from rflearning_2 import train2
from rflearning_3 import train3
from planning import planning


def talker():
    pub1 = rospy.Publisher('/mybot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/mybot/joint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/mybot/joint3_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    train3(pub1, pub2, pub3)
	
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
