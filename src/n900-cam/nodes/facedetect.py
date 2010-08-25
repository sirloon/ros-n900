#!/usr/bin/env python
import roslib
roslib.load_manifest('n900-cam')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0

class face_detect(object):

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    ##self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
    self.image_sub = rospy.Subscriber("/gscam/image_raw",Image,self.callback)
    self.cascade = cv.Load("data/haarcascade_frontalface_alt.xml")

  def detect_and_draw(self,img):
    # allocate temporary images
    gray = cv.CreateImage((img.width,img.height), 8, 1)
    small_img = cv.CreateImage((cv.Round(img.width / image_scale),
  	  	       cv.Round (img.height / image_scale)), 8, 1)
  
    # convert color input image to grayscale
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
  
    # scale input image for faster processing
    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)
  
    cv.EqualizeHist(small_img, small_img)
  
    if(self.cascade):
        t = cv.GetTickCount()
        faces = cv.HaarDetectObjects(small_img, self.cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_size)
        t = cv.GetTickCount() - t
        print "detection time = %gms" % (t/(cv.GetTickFrequency()*1000.))
        if faces:
            for ((x, y, w, h), n) in faces:
                # the input to cv.HaarDetectObjects was resized, so scale the 
                # bounding box of each face and convert it to two CvPoints
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
      
    return img



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    cv_image = self.detect_and_draw(cv_image)

    cv.ShowImage("Image window", cv_image)
    cv.WaitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = face_detect()
  rospy.init_node('face_detect', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
