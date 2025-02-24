#!/usr/bin/env python

import rospy

def main():
    rospy.init_node('gate_manuscript_testing', anonymous=True)
    rospy.loginfo("Started gate_manuscript_testing")
    rospy.spin()  # Keeps the node running

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
