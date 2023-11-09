import cv2 
import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image,Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from numpy import asarray

# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL

def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0] #get the file of the
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)

def ReadIMU(IMUFile):
    '''return IMU data and timestamp of IMU'''
    IMUfp = open(IMUFile,'r')
    IMULines = IMUfp.readlines()
    #all = IMUDatas.readlines()
    IMUDatas = {}
    for l in IMULines:
        if l[0] == "#":
            continue;
        items = l.rstrip('\n').split(' ')
        IMUDatas[items[0]] = items[1:]
    
    IMUfp.close()
    return IMUDatas 

def ReadImages(assocoations):
   assofp = open(assocoations, 'r')
   asso = assofp.readlines()
   RGBImages = {}
   depthImages = {}
   for l in asso:
       if l[0] == "#":
           continue;
       items = l.rstrip('\n').split(' ')
       RGBImages[items[0]] = items[1]
       depthImages[items[2]] = items[3]

   assofp.close()
   return RGBImages, depthImages

def CreateBag(args):#assocoations, imu, output_bag_name
    '''read assocoations.txt'''
    RGBImages,depthImages = ReadImages(args[1])

    IMUDatas = ReadIMU(args[2]) #the url of IMU data

    '''Creates a bag file with camera images'''
    if not os.path.exists(args[3]):
       os.system(r'touch %s' % args[3])
    else:
       os.system(r'rm %s' % args[3])
       os.system(r'touch %s' % args[3])

    bagName = rosbag.Bag(args[3], 'w')

    try:
        for it, iData in IMUDatas.items():
            imu = Imu()
            imuStamp = rospy.rostime.Time.from_sec(float(it))
            #angular_v = Vector3()
            linear_a = Vector3()
            #angular_v.x = float(iData[0])
            #angular_v.y = float(iData[1])
            #angular_v.z = float(iData[2])
            linear_a.x = float(iData[0])
            linear_a.y = float(iData[1])
            linear_a.z = float(iData[2])
            imu.header.stamp = imuStamp
            #imu.angular_velocity = angular_v
            imu.linear_acceleration = linear_a

            bagName.write("/imu",imu,imuStamp)

        br = CvBridge()

        for imt, img in RGBImages.items():
            #img = args[2] + img; 
            print("Adding %s" % img)

            cv_image = cv2.imread(img)

            Stamp = rospy.rostime.Time.from_sec(float(imt))

            '''set image information '''
            Img = br.cv2_to_imgmsg(cv_image)
            Img.header.stamp = Stamp
            Img.header.frame_id = "camera"

            '''for mono8'''
            Img.encoding = "rgb8"
            bagName.write('/camera/rgb/image_color', Img, Stamp)

        for dt, dimg in depthImages.items():
            #dimg = args[2] + dimg; 
            print("Adding %s" % dimg)

            cv_image = cv2.imread(dimg, cv2.IMREAD_ANYDEPTH)

            '''set image information '''
            Stamp = rospy.rostime.Time.from_sec(float(dt))

            '''set image information '''
            dImg = br.cv2_to_imgmsg(cv_image)
            dImg.header.stamp = Stamp
            dImg.header.frame_id = "camera"

            #dImg.encoding = "32FC1"

            bagName.write('/camera/depth/image', dImg, Stamp)

    finally:
        bagName.close()

if __name__ == "__main__":
    print(sys.argv)

    if len(sys.argv) < 4:
        print("Usage:\n\t python generate_bags.py /path/assocoations.txt /path/accelerometer.txt output.bag")
        exit(1)

    CreateBag(sys.argv)

