#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import timeit
import time
import tf
import message_filters as mf

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32
#from sensor_msgs.msg import DisparityImage
from cv_bridge import CvBridge

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

class RosDepth:

    def __init__(self):

        #initial disparity variable values
        self.speckWin = 300
        self.speckRan = 10
        self.maxDiff = 1
        self.ndisparities = 16
        self.minDisp = 6
        self.maxDisp = 72
        self.SADwind = 6
        self.uniq = 10
        self.DP = 0
        self.P1 = 8 * 3 * 3 ** 2
        self.P2 = 1000

        makePly = 0

        rospy.init_node('stereovision')

        cv2.namedWindow("disparity", 0)

        #bars used to adjust disparity values
        cv2.createTrackbar("minDisp", "disparity", 6, 100, self.updateMin)
        cv2.createTrackbar("maxDisp", "disparity", 72, 150, self.updateMax)
        cv2.createTrackbar('SAD', "disparity", 6, 32, self.updateSAD)
        cv2.createTrackbar('uniqueness', "disparity", 10, 50, self.updateUnique)
        #cv2.createTrackbar('fullDP', "disparity", 0, 1, self.updateDP)
        cv2.createTrackbar('p1', "disparity", 24*3**2, 800, self.updateP1)
        cv2.createTrackbar('p2', "disparity", 1000, 2500, self.updateP2)
        cv2.createTrackbar('speckWin', "disparity", 300, 1000, self.updateSpeckWin)
        cv2.createTrackbar('speckRan', "disparity", 19, 100, self.updateSpeckRan)
        cv2.createTrackbar('maxDiff', "disparity", 1, 500, self.updateMaxDiff)
        #cv2.createTrackbar("ndisparities", "disparity", 1, 100, self.updatenDisp)
        #cv2.createTrackbar('SAD', "disparity", 1, 20, self.updateSAD)
        
        #used to make sure that both left and right image has been received
        #before calculating disparity
        self.L = 0
        self.R = 0

        #subscribers and publishers
        camL = rospy.Subscriber('camera/left/image_rect_color/compressed', CompressedImage, self.callbackL, queue_size = 1)
        camR = rospy.Subscriber('camera/right/image_rect_color/compressed', CompressedImage, self.callbackR, queue_size = 1)
        info = rospy.Subscriber('camera/left/camera_info', CameraInfo, self.callbackInfo, queue_size = 1)
        self.depthPub = rospy.Publisher('camera/sync/selfDepth', Image, queue_size=1)
        self.infoPub = rospy.Publisher('camera/sync/camera_info', CameraInfo, queue_size=1)
        self.lowPub = rospy.Publisher('camera/sync/lowRes', Image, queue_size=1)

        #used to convert from cv2 to ros.msg
        self.bridge = CvBridge()

        #tf frames
        lefttf = tf.TransformBroadcaster()
        righttf = tf.TransformBroadcaster()

        while True:
            #if both left and right images received
            if self.L and self.R:
                #used to calculate fps (frames per second)
                self.start = timeit.default_timer()

                #calculate disparity
                self.stereoDisp()

                #Get new images
                self.L = 0
                self.R = 0

                #send tf frames, using random numbers for testing, please change as necessary
                lefttf.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 1.57),
                    rospy.Time.now(),
                    "lefttf",
                    "world")
                righttf.sendTransform((.075, 0, 0), (0,0,0,0),
                    rospy.Time.now(),
                    "righttf",
                    "lefttf")

                #exit on 'q', make a .ply file that can be viewed in Blender or Meshlab
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    if makePly:
                        self.out_points = self.out_points.reshape(-1, 3)
                        self.out_colors = self.out_colors.reshape(-1, 3)
                        self.out_points = np.hstack([self.out_points, self.out_colors])
                        with open('out.ply', 'w') as f:
                            f.write(ply_header % dict(vert_num=len(self.out_points)))
                            np.savetxt(f, self.out_points, '%f %f %f %d %d %d')
                        print '%s saved' % 'out.ply'
                        break
        camL.release()
        camR.release()
        cv2.destroyAllWindows()

    #callback functions for subscribers and trackbars
    def callbackInfo(self, ros_data):
        self.camInfo = ros_data

    def callbackL(self, ros_data):
        #print 'test L'
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.frameL= cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.L = 1

    def callbackR(self, ros_data):
        #print 'test R'
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.frameR = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.R = 1

    def updatenDisp(self, val):
        self.ndisparities = val
        remainder = self.ndisparities % 16
        print remainder
        if (remainder!= 0):
            self.ndisparities = self.ndisparities + (16-remainder)

        print self.ndisparities
        
    def updateSpeckWin(self, val):
        self.speckWin = val  
            
    def updateSpeckRan(self, val):
        self.speckRan = val

    def updateMaxDiff(self, val):
        self.maxDiff = val

    def updateMin(self, val):
        self.minDisp = val

    def updateMax(self, val):
        self.maxDisp = val

    def updateSAD(self, val):
        self.SADwind = val
        self.P1 = 8 * 3 * val * 2
        self.P2 = 32 * 3 * val * 2

    def updateUnique(self, val):
        self.uniq = val

    def updateP1(self, val):
        self.P1 = val

    def updateP2(self, val):
        self.P2 = val

    def updateDP(self, val):
        self.DP = val

    #save the .ply created on exit
    def write_ply(self, fn, verts, colors):
        verts = verts.reshape(-1, 3)
        colors = colors.reshape(-1, 3)
        verts = np.hstack([verts, colors])
        with open(fn, 'w') as f:
            f.write(self.ply_header % dict(vert_num=len(verts)))
            np.savetxt(f, verts, '%f %f %f %d %d %d')

    #calculate the disparity
    def stereoDisp(self):

        #make sure variables are not out of range
        if (self.maxDisp - self.minDisp >= 16):
            self.numDisp =  self.maxDisp - self.minDisp
            self.numDisp -= int(self.numDisp%16)

        if (self.SADwind < 5):
            self.SADwind = 5
        if (self.SADwind % 2 != 1):
            self.SADwind = self.SADwind + 1

        #used to calculate disparity
        stereo = cv2.StereoSGBM(minDisparity = self.minDisp,
                                numDisparities = self.numDisp,
                                SADWindowSize = self.SADwind,
                                uniquenessRatio = self.uniq,
                                speckleWindowSize = self.speckWin,
                                speckleRange = self.speckRan,
                                disp12MaxDiff = self.maxDiff,
                                P1 = self.P1,
                                P2 = self.P2,
                                fullDP = self.DP
        )

        #downsize source images, speeds up calculations by a large magnitude
        frameLS = cv2.resize(self.frameL,None,fx=.5, fy=.5)
        frameRS = cv2.resize(self.frameR,None,fx=.5, fy=.5)

        #calculate the actual disparity
        disp = stereo.compute(frameLS, frameRS).astype(np.float32) / 16.0

        #Calculate the point cloud to create the .ply
        #can be commented out to speed up overall program speed
        #Seperate ros pkg can calculate a PointCloud2 using the disparity image
        #(See stereo.launch, depth_image_proc)
        '''
        h, w = frameLS.shape[:2]
        f = 0.8*w                          # guess for focal length
        Q = np.float32([[1, 0, 0, -0.5*w],
                        [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                        [0, 0, 0,     -f], # so that y-axis looks up
                        [0, 0, 1,      0]])
        points = cv2.reprojectImageTo3D(disp, Q)
        colors = cv2.cvtColor(frameLS, cv2.COLOR_BGR2RGB)
        mask = disp > disp.min()
        self.out_points = points[mask]
        self.out_colors = colors[mask]
        out_fn = 'out.ply'
        '''
        
        #show the left and right source frame for reference
        cv2.imshow('Left', frameLS)
        cv2.imshow('Right', frameRS)

        #depth is represented as floats rather than integers
        disp = (disp-self.minDisp)/self.numDisp

        #depth_image_proc uses brighter == farther away rather than brighter == closer
        #so resulting depth image must be inverted
        invdisp = (disp-self.minDisp)/self.numDisp
        invdisp = 1 - invdisp
        #threshold the pure white back to black as no disparity was seen at those points
        ret, invdisp = cv2.threshold(invdisp,.98,255,cv2.THRESH_TOZERO_INV)

        #show the disparity/inverse disparity images
        cv2.imshow('disparity', disp)
        cv2.imshow('inverse', invdisp)

        #convert to ros.msgs
        dispPub = self.bridge.cv2_to_imgmsg(disp)
        lowRes = self.bridge.cv2_to_imgmsg(frameLS, 'bgr8')

        #convert disparity image to uint16 for use by depth_image_proc
        #adjust max_m as needed
        cv_image_tmp = self.bridge.imgmsg_to_cv2(dispPub)
        height, width = cv_image_tmp.shape
        max_m = 2.0 #max distance that can be seen, should be adjusted
        cv_image_tmp = np.clip(cv_image_tmp,0,max_m) ## Filter all values > 5.0 m
        scaling_factor = 65535/max_m
        cv_image_tmp = cv_image_tmp*scaling_factor #scaling the image to [0;65535]
        cv_image = np.array(cv_image_tmp,dtype=np.uint16) ## Creating the cv2 image

        dispPub = self.bridge.cv2_to_imgmsg(cv_image)

        #publish images, set all the time stamps to the same value
        #since depth_image_proc wants "synchronized" messages
        time = rospy.Time.now()
        dispPub.header.stamp = time
        self.camInfo.header.stamp = time
        lowRes.header.stamp = time
        lowRes.header.frame_id = "lefttf"
        dispPub.header.frame_id = "lefttf"

        self.depthPub.publish(dispPub)
        self.infoPub.publish(self.camInfo)
        self.lowPub.publish(lowRes)

        end = timeit.default_timer()
        #print framerate
        print 1/ (end - self.start)


depthImage = RosDepth()
rospy.spin()
#rosImage.stereoDisp()