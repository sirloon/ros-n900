#!/usr/bin/env python
import roslib
roslib.load_manifest('n900_cam')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

min_size = (90,90)  # adjust high/low whether detection or recognition is wanted
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0

class face_detect(object):

  def __init__(self):
    self.face_pub = rospy.Publisher("/face_topic",Image)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    ##self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
    # subscribe to gray scaled images, because AT&T db are gray images 
    # (so no need to convert)
    self.image_sub = rospy.Subscriber("/gscam/image_mono",Image,self.callback)
    self.cascade = cv.Load("data/haarcascade_frontalface_alt.xml")

  def detect_and_draw(self,img):
    # allocate temporary images
    small_img = cv.CreateImage((cv.Round(img.width / image_scale),
  	  	       cv.Round (img.height / image_scale)), 8, 1)
  
    # scale input image for faster processing
    cv.Resize(img, small_img, cv.CV_INTER_LINEAR)
    # since image is resized, we need to resize detection area as well
    min_resize = tuple(map(lambda x: x/image_scale,min_size))
  
    cv.EqualizeHist(small_img, small_img)
  
    if(self.cascade):
        t = cv.GetTickCount()
        faces = cv.HaarDetectObjects(small_img, self.cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_resize)
        t = cv.GetTickCount() - t
        if faces:
            print "detection time = %gms" % (t/(cv.GetTickFrequency()*1000.))
            for ((x, y, w, h), n) in faces:
                # the input to cv.HaarDetectObjects was resized, so scale the 
                # bounding box of each face and convert it to two CvPoints

                # add some more area because detected faces are little narrow (eg. hair missing)
                adjust = 0.1    # 10% more
                x = x * (1-adjust) * image_scale
                y = y * (1-adjust) * image_scale
                w = w * (1+adjust) * image_scale
                h = h * (1+adjust) * image_scale

                pt1 = (x,y)
                pt2 = (x + w, y + h)

                face_img = cv.GetSubRect(img,(x,y,w,h))
                cv.EqualizeHist(face_img,face_img)  # this should help having consistent data (or not...)
                # publish face before adding rectangle
                self.face_pub.publish(self.bridge.cv_to_imgmsg(face_img,"mono8"))

                cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
      
    return img



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
    except CvBridgeError, e:
      print e

    cv_image = self.detect_and_draw(cv_image)

    cv.ShowImage("Image window", cv_image)
    cv.WaitKey(3)


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
