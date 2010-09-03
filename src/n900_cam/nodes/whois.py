#!/usr/bin/env python
import roslib; roslib.load_manifest('n900_cam')
import rospy
from std_msgs.msg import String
import os, time

last_whois = time.time()

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
    name = "unknown"
    if data.data == "1":
        name = "franck"
    elif data.data == "2":
        name = "seb"
    elif data.data == "3":
        name = "antoine"

    # don't say your name 10 times a sec...
    global last_whois
    if time.time() > last_whois + 2:
        os.system("espeak 'hello %s'" % name)
        last_whois = time.time()
    else:
        rospy.loginfo("It's %s, just said your name @ %s, come back later" % (time.time(),last_whois))

def listener():
    rospy.init_node('whois', anonymous=True)
    rospy.Subscriber("/whois", String, callback)
    rospy.loginfo("Show me your face...")
    rospy.spin()

if __name__ == '__main__':
    listener()
