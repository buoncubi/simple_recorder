#!/usr/bin/env python
import roslib
import sys 
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import os

class recorder:

    BASE_PATH = ""#"/home/luca-bnc/ros_ws/src/simple_recorder/log/test1/"
    IMG_NAME = "-snapshot.jpg"
    LOG_NAME = "positions.txt"
    LOG_PATH = ""
    CONNECTOR_OFSET_X = 0.15 # +- 0.02 [m]
    CONNECTOR_OFSET_Y = 0.09 # +- 0.02 [m]    
    FREQUENCY = 1/0.4 #Hz
    SEP = ", "
    SEP_LINE = ";\n"

    def __init__(self, basePath):
        self.BASE_PATH = basePath
        self.LOG_PATH = basePath + self.LOG_NAME
            
        self.bridge = CvBridge()
        self.logfFile = self.LOG_PATH

        self.writeToFile("ID (" + str( self.FREQUENCY) + "Hz)" + self.SEP + "Table X" + self.SEP + "Table Y" + self.SEP + "Connector1 X" + self.SEP + "Connector1 Y" + self.SEP + "Connector2 X" + self.SEP + "Connector2 Y" + self.SEP + "Connector3 X" + self.SEP + "Connector3 Y" + self.SEP + "Connector4 X" + self.SEP + "Connector4 Y" + self.SEP + "Leg1 X" + self.SEP + "Leg1 Y" + self.SEP + "Leg2 X" + self.SEP + "Leg2 Y" + self.SEP + "Leg3 X" + self.SEP + "Leg3 Y" + self.SEP + "Leg4 X" + self.SEP + "Leg4 Y" + self.SEP + "Container1 X" + self.SEP + "Container1 Y" + self.SEP + "Container2 X" + self.SEP + "Container2 Y" + self.SEP + "Container3 X" + self.SEP + "Container3 Y" + self.SEP + "Container4 X" + self.SEP + "Container4 Y" + self.SEP + "Container5 X" + self.SEP + "Container5 Y" + self.SEP + "Pen1 X" + self.SEP + "Pen1 Y" + self.SEP + "Pen2 X" + self.SEP + "Pen2 Y" + self.SEP + self.SEP_LINE)
        

        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.imageCallBack)
        self.table_sub = rospy.Subscriber("/Rigid_Body/Table/pose", PoseStamped, self.tableCallBack)
        self.leg1_sub = rospy.Subscriber("/Rigid_Body/Leg_1/pose", PoseStamped, self.leg1CallBack)
        self.leg2_sub = rospy.Subscriber("/Rigid_Body/Leg_2/pose", PoseStamped, self.leg2CallBack)
        self.leg3_sub = rospy.Subscriber("/Rigid_Body/Leg_3/pose", PoseStamped, self.leg3CallBack)
        self.leg4_sub = rospy.Subscriber("/Rigid_Body/Leg_4/pose", PoseStamped, self.leg4CallBack)
        self.container1_sub = rospy.Subscriber("/Rigid_Body/Container_1/pose", PoseStamped, self.container1CallBack)
        self.container2_sub = rospy.Subscriber("/Rigid_Body/Container_2/pose", PoseStamped, self.container2CallBack)
        self.container3_sub = rospy.Subscriber("/Rigid_Body/Container_3/pose", PoseStamped, self.container3CallBack)
        self.container4_sub = rospy.Subscriber("/Rigid_Body/Container_4/pose", PoseStamped, self.container4CallBack)
        self.container5_sub = rospy.Subscriber("/Rigid_Body/Container_5/pose", PoseStamped, self.container5CallBack)
        self.pen1_sub = rospy.Subscriber("/Rigid_Body/Pen_1/pose", PoseStamped, self.pen1CallBack)
        self.pen2_sub = rospy.Subscriber("/Rigid_Body/Pen_2/pose", PoseStamped, self.pen2CallBack)
        
        # data for saving the rigid bodies
        self.tablePose = None
        self.connector1Pose = None
        self.connector2Pose = None        
        self.connector3Pose = None        
        self.connector4Pose = None        
        self.leg1Pose = None
        self.leg2Pose = None        
        self.leg3Pose = None        
        self.leg4Pose = None   
        self.container1Pose = None
        self.container2Pose = None
        self.container3Pose = None
        self.container4Pose = None
        self.container5Pose = None
        self.pen1Pose = None
        self.pen2Pose = None


        self.tableFlag = False
        self.leg1Flag = False
        self.leg2Flag = False       
        self.leg3Flag = False       
        self.leg4Flag = False  
        self.container1Flag = False
        self.container2Flag = False
        self.container3Flag = False
        self.container4Flag = False
        self.container5Flag = False
        self.pen1Flag = False
        self.pen2Flag = False

        
        # data for saving the image
        self.cnt = 0
        self.img = None

    def objDisappeared(self,ref,data):
        if ref is not None:
            if self.getX( ref) == self.getX(data) and self.getY( ref) == self.getY(data):
                return True
        return False

    def tableCallBack(self,data):
        if self.objDisappeared( self.tablePose, data) == True:
            self.tableFlag = False
            return
        self.tablePose = data
        self.connector1Pose = PoseStamped();
        self.connector1Pose.pose.position.x = data.pose.position.x + self.CONNECTOR_OFSET_X
        self.connector1Pose.pose.position.y = data.pose.position.y + self.CONNECTOR_OFSET_Y
        self.connector2Pose = PoseStamped();
        self.connector2Pose.pose.position.x = data.pose.position.x - self.CONNECTOR_OFSET_X
        self.connector2Pose.pose.position.y = data.pose.position.y + self.CONNECTOR_OFSET_Y
        self.connector3Pose = PoseStamped();
        self.connector3Pose.pose.position.x = data.pose.position.x - self.CONNECTOR_OFSET_X
        self.connector3Pose.pose.position.y = data.pose.position.y - self.CONNECTOR_OFSET_Y
        self.connector4Pose = PoseStamped();
        self.connector4Pose.pose.position.x = data.pose.position.x + self.CONNECTOR_OFSET_X
        self.connector4Pose.pose.position.y = data.pose.position.y - self.CONNECTOR_OFSET_Y
        self.tableFlag = True

    def leg1CallBack(self,data):
        if self.objDisappeared( self.leg1Pose, data) == True:
            self.leg1Flag = False
            return
        self.leg1Pose = data
        self.leg1Flag = True
        
    def leg2CallBack(self,data):
        if self.objDisappeared( self.leg2Pose, data) == True:
            self.leg2Flag = False
            return    
        self.leg2Pose = data        
        self.leg2Flag = True

    def leg3CallBack(self,data):
        if self.objDisappeared( self.leg3Pose, data) == True:
            self.leg3Flag = False
            return
        self.leg3Pose = data        
        self.leg3Flag = True
        
    def leg4CallBack(self,data):
        if self.objDisappeared( self.leg4Pose, data) == True:
            self.leg4Flag = False
            return
        self.leg4Pose = data
        self.leg4Flag = True
        
    def container1CallBack(self,data):
        if self.objDisappeared( self.container1Pose, data) == True:
            self.container1Flag = False
            return
        self.container1Pose = data     
        self.container1Flag = True           

    def container2CallBack(self,data):
        if self.objDisappeared( self.container2Pose, data) == True:
            self.container2Flag = False
            return
        self.container2Pose = data     
        self.container2Flag = True   

    def container3CallBack(self,data):
        if self.objDisappeared( self.container3Pose, data) == True:
            self.container3Flag = False
            return
        self.container3Pose = data        
        self.container3Flag = True

    def container4CallBack(self,data):
        if self.objDisappeared( self.container4Pose, data) == True:
            self.container4Flag = False
            return
        self.container4Pose = data     
        self.container4Flag = True   
        
    def container5CallBack(self,data):
        if self.objDisappeared( self.container5Pose, data) == True:
            self.container5Flag = False
            return
        self.container5Pose = data
        self.container5Flag = True
        
    def pen1CallBack(self,data):
        if self.objDisappeared( self.pen1Pose, data) == True:
            self.pen1Flag = False
            return
        self.pen1Pose = data
        self.pen1Flag = True
        
    def pen2CallBack(self,data):
        if self.objDisappeared( self.pen2Pose, data) == True:
            self.pen2Flag = False
            return
        self.pen2Pose = data
        self.pen2Flag = True
                
    def imageCallBack(self,data):
        self.img = data
            
    def saveImg(self):
        if self.img is None:
            print('none')
            return
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(self.img, "bgr8")
        except CvBridgeError, e:
            print e
        else:
            (h, w) = cv2_img.shape[:2]
            crop_img = cv2_img[h/2.7:h, 0:w-130]
            filePath = self.BASE_PATH + str(self.cnt) + self.IMG_NAME;
            cv2.imwrite(  filePath, crop_img)
            print "image [" + str(self.cnt) + "] saved in \'" + filePath + "\'"
            
    def storeRigidBody(self):
        tableStr = self.SEP + self.SEP
        connector1Str = self.SEP + self.SEP
        connector2Str = self.SEP + self.SEP
        connector3Str = self.SEP + self.SEP
        connector4Str = self.SEP + self.SEP
        if self.tableFlag == True:
            tableStr = str(self.getX( self.tablePose)) + self.SEP + str(self.getY(self.tablePose)) + self.SEP
            print "table: " + tableStr
            
            connector1Str = str(self.getX( self.connector1Pose)) + self.SEP + str(self.getY( self.connector1Pose)) + self.SEP
            connector2Str = str(self.getX( self.connector2Pose)) + self.SEP + str(self.getY( self.connector2Pose)) + self.SEP
            connector3Str = str(self.getX( self.connector3Pose)) + self.SEP + str(self.getY( self.connector3Pose)) + self.SEP
            connector4Str = str(self.getX( self.connector4Pose)) + self.SEP + str(self.getY( self.connector4Pose)) + self.SEP
            print "connector1: " + connector1Str
            print "connector2: " + connector2Str
            print "connector3: " + connector3Str
            print "connector4: " + connector4Str
            
        leg1Str = self.SEP + self.SEP
        if self.leg1Flag == True:
            leg1Str = str(self.getX( self.leg1Pose)) + self.SEP + str(self.getY( self.leg1Pose)) + self.SEP
            print "leg1: " + leg1Str
            
        leg2Str = self.SEP + self.SEP
        if self.leg2Flag == True:
            leg2Str = str(self.getX( self.leg2Pose)) + self.SEP + str(self.getY( self.leg2Pose)) + self.SEP
            print "leg2: " + leg2Str
            
        leg3Str = self.SEP + self.SEP
        if self.leg3Flag == True:
            leg3Str = str(self.getX( self.leg3Pose)) + self.SEP + str(self.getY( self.leg3Pose)) + self.SEP
            print "leg3: " + leg3Str

        leg4Str = self.SEP + self.SEP           
        if self.leg4Flag == True:
            leg4Str = str(self.getX( self.leg4Pose)) + self.SEP + str(self.getY( self.leg4Pose)) + self.SEP
            print "leg4: " + leg4Str
            
        container1Str = self.SEP + self.SEP
        if self.container1Flag == True:
            container1Str = str(self.getX( self.container1Pose)) + self.SEP + str(self.getY( self.container1Pose)) + self.SEP
            print "container1: " + container1Str
            
        container2Str = self.SEP + self.SEP
        if self.container2Flag == True:
            container2Str = str(self.getX( self.container2Pose)) + self.SEP + str(self.getY( self.container2Pose)) + self.SEP
            print "container2: " + container2Str 
        
        container3Str = self.SEP + self.SEP 
        if self.container3Flag == True:
            container3Str = str(self.getX( self.container3Pose)) + self.SEP + str(self.getY( self.container3Pose)) + self.SEP
            print "container3: " + container3Str 
            
        container4Str = self.SEP + self.SEP 
        if self.container4Flag == True:
            container4Str = str(self.getX( self.container4Pose)) + self.SEP + str(self.getY( self.container4Pose)) + self.SEP
            print "container4: " + container4Str 

        container5Str = self.SEP + self.SEP
        if self.container5Flag == True:
            container5Str = str(self.getX( self.container5Pose)) + self.SEP + str(self.getY( self.container5Pose)) + self.SEP
            print "container5: " + container5Str
            
        pen1Str = self.SEP + self.SEP
        if self.pen1Flag == True:
            pen1Str =  str(self.getX( self.pen1Pose)) + self.SEP + str(self.getY( self.pen1Pose)) + self.SEP
            print "pen1: " + pen1Str

        pen2Str = self.SEP + self.SEP_LINE
        if self.pen2Flag == True:
            pen2Str = str(self.getX( self.pen2Pose)) + self.SEP + str(self.getY( self.pen2Pose)) + self.SEP_LINE
            print "pen2: " + pen2Str 
        print "--------------------------------------"
        
        self.writeToFile( str(self.cnt) + self.SEP + tableStr + connector1Str + connector2Str + connector3Str + connector4Str + leg1Str + leg2Str + leg3Str + leg4Str + container1Str + container2Str + container3Str + container4Str + container5Str + pen1Str + pen2Str)

        print "logs saved in \'" + self.LOG_PATH + "\'"

    def writeToFile(self, content):
        filepath = os.path.join( self.BASE_PATH, self.LOG_NAME)
        if not os.path.exists( self.BASE_PATH):
            os.makedirs( self.BASE_PATH)
        with open(filepath, "a") as f:
            f.write( content)

    def getX(self,data):
        if data is None:
            return None
        return data.pose.position.x

    def getY(self,data):
        if data is None:
            return None    
        return data.pose.position.y

    def clean(self):
#        self.tablePose = None
#        self.connector1Pose = None
#        self.connector2Pose = None
#        self.connector3Pose = None
#        self.connector4Pose = None
#        self.leg1Pose = None
#        self.leg2Pose = None
#        self.leg3Pose = None
#        self.leg4Pose = None
#        self.container1Pose = None
#        self.container2Pose = None
#        self.container3Pose = None
#        self.container4Pose = None
#        self.container5Pose = None
#        self.pen1Pose = None
#        self.pen2Pose = None
        self.img = None

def main(args):
    BASE_PATH = rospy.get_param( "/Recorder/base_path")
    ic = recorder(BASE_PATH)
    rospy.init_node('recorder', anonymous=True)
    try:
        r = rospy.Rate( ic.FREQUENCY)
        while not rospy.is_shutdown():
            ic.saveImg()
            ic.storeRigidBody()
            ic.clean()
            ic.cnt = ic.cnt + 1;
            r.sleep()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)


