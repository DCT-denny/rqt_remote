#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Version PYQT5

import os
import rospy
import rospkg
import math

from PyQt5 import QtCore
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget,QApplication
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from std_msgs.msg import Float64,Int16MultiArray,Int16,String,UInt8MultiArray,Float64MultiArray
from sensor_msgs.msg import LaserScan,Imu
import tf
import numpy as np
import cv2 as cv

class ROSdata(QWidget):

    Mode_Start_AUTO = 1
    Mode_Start_Remote = 2
    Mode_Stop = -1
    updata_laser = QtCore.pyqtSignal()
    updata_imu = QtCore.pyqtSignal()
    #updata_arm = QtCore.pyqtSignal()
    def __init__(self,context):
        super(ROSdata, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_remote'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self)

        self.cnt=0
        self.lidar_ave=0
        self._sensor_value = [0, 0, 0, 0, 0, 0]
        self._arm_value=[0,0,0,0,0,0,0,0,0]
        self._wallfuzzy_value =[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.lidar_msg = np.zeros(684,dtype=np.float)
        """
        self.image_list=[QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range1.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range2.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range3.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range4.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range5.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range6.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range7.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range8.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range9.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range10.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range11.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range12.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range13.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range14.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range15.png"),
                        QPixmap("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range16.png")]
        """

        self.subscriber_group()
        self.publisher_group()
        self.pushButton_start.clicked.connect(self.robot_start)

        self.updata_laser.connect(self.slotlaser)
        self.updata_imu.connect(self.slotimu)


    def publisher_group(self):
        self.task_pub = rospy.Publisher('/task_num', Int16, queue_size=10)
        self.start_pub = rospy.Publisher('/robot_start', Int16, queue_size=10)


    def subscriber_group(self):
        rospy.Subscriber("/scan",LaserScan, self.laser_callback)
        rospy.Subscriber("/light_err",Float64, self.callback)




        rospy.Subscriber("/imu/data",Imu, self.imu_callback)

        rospy.Subscriber("/move_it",Int16MultiArray, self.move_callback)
        rospy.Subscriber("/linesss",Int16MultiArray, self.view3wall_callback) #test view 3 wall

    def robot_start(self):
        if self.radioButton_AUTO.isChecked():
            self.start_pub.publish(self.Mode_Start_AUTO)
        elif self.radioButton_Remote.isChecked():
            self.start_pub.publish(self.Mode_Start_Remote)
        elif self.radioButton_Stop.isChecked():
            self.start_pub.publish(self.Mode_Stop)



    def callback(self,err):
        #print("err" + str(err.data))
        #self.lcdNumber.display(err.data)
        pass

    def view3wall_callback(self,point):
        img = np.zeros((400,400,3), np.uint8)
        _x1=np.zeros(16,dtype=np.int)
        _x2=np.zeros(16,dtype=np.int)
        _y1=np.zeros(16,dtype=np.int)
        _y2=np.zeros(16,dtype=np.int)
        for i in range(0,16):
            _x1[i] = point.data[4*i]
            _y1[i] = point.data[4*i+1]
            _x2[i] = point.data[4*i+2]
            _y2[i] = point.data[4*i+3]
        for i in range(0,16):
            cv.line(img,(_x1[i],_y1[i]),(_x2[i],_y2[i]),(255,0+i*10,0),3)
        cv.line(img,(200,200),(210,200),(0,255,0),3)
        cv.line(img,(200,200),(200,190),(0,255,0),3)

        self.height, self.width, self.bytesPerComponent = img.shape
        self.bytesPerLine = 3 *self.width
        QImg = QImage(img.data, self.width, self.height, self.bytesPerLine,QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(QImg)
        self.label_qimage.setPixmap(pixmap)


    def move_callback(self,move_msg):
        self.lcdNumber_Lspeed.display(move_msg.data[0])
        self.lcdNumber_Rspeed.display(move_msg.data[1])

    def laser_callback(self,msg):
        """
        if math.isnan(msg.ranges[85]):
            pass
        else:
            self.lcdNumber_right_scan.display(msg.ranges[85]*100)
        if math.isnan(msg.ranges[341]):
            pass
        else:
            self.lcdNumber_forward_scan.display(msg.ranges[341]*100)
        if math.isnan(msg.ranges[597]):
            pass
        else:
            self.lcdNumber_left_scan.display(msg.ranges[597]*100)
        """
        for i in range(1,684):
            self.lidar_msg[i] = msg.ranges[i]
        """
        for i in range(0,16):
            for j in range(0,43):
                lidar_ave = lidar_ave + lidar_msg[i*42+(j+1)]
            if lidar_ave <0.3:
                self.label_pic.setPixmap(self.image_list[i+1])
        """
        self.updata_laser.emit()

    def slotlaser(self):
        if math.isnan(self.lidar_msg[85]):
            pass
        else:
            self.lcdNumber_right_scan.display(self.lidar_msg[85]*100)
        if math.isnan(self.lidar_msg[341]):
            pass
        else:
            self.lcdNumber_forward_scan.display(self.lidar_msg[341]*100)
        if math.isnan(self.lidar_msg[597]):
            pass
        else:
            self.lcdNumber_left_scan.display(self.lidar_msg[597]*100)
        self.cnt=0

        pts = np.array([[182,184],[182,184],[330,221],[314,262]], np.int32)
        pts2 = np.array([[180,182],[180,179],[337,182],[330,221]], np.int32)
        pts3 = np.array([[180,179],[180,177],[331,134],[337,182]], np.int32)
        pts4 = np.array([[180,177],[179,174],[316,95],[331,134]], np.int32)
        pts5 = np.array([[179,174],[178,171],[288,60],[316,95]], np.int32)
        pts6 = np.array([[178,171],[176,170],[254,34],[288,60]], np.int32)
        pts7 = np.array([[176,170],[173,170],[212,16],[254,34]], np.int32)
        pts8 = np.array([[173,170],[171,170],[166,11],[212,16]], np.int32)
        pts9 = np.array([[171,170],[169,170],[127,16],[166,11]], np.int32)
        pts10 = np.array([[169,170],[166,170],[87,34],[127,16]], np.int32)
        pts11 = np.array([[166,170],[163,171],[52,60],[87,34]], np.int32)
        pts12 = np.array([[163,171],[162,174],[25,94],[52,60]], np.int32)
        pts13 = np.array([[162,174],[162,177],[10,135],[25,94]], np.int32)
        pts14 = np.array([[162,177],[162,179],[2,182],[10,135]], np.int32)
        pts15 = np.array([[162,179],[162,182],[9,221],[2,182]], np.int32)
        pts16 = np.array([[162,182],[163,184],[25,264],[9,221]], np.int32)

        pts_arr=[pts,pts2,pts3,pts4,pts5,pts6,pts7,pts8,pts9,pts10,pts11,pts12,pts13,pts14,pts15,pts16]
        for i in range(0,16):
            pts_arr[i] = pts_arr[i].reshape((-1,1,2))

        img2 = cv.imread("/home/denny3/new_work/src/rqt_remote/lidar_pic/lidar_range.png")
        self.height, self.width, self.bytesPerComponent = img2.shape
        self.bytesPerLine = 3 *self.width




        for i in range(0,16):
            self.lidar_ave=0
            for j in range(0,43):
                self.lidar_ave = self.lidar_ave + self.lidar_msg[i*42+(j+1)]
            self.lidar_ave=self.lidar_ave/43
            if self.lidar_ave <0.3:
                cv.fillPoly(img2,[pts_arr[i]],(255,255,0))

                    #self.cnt=self.cnt+1
                #if self.cnt ==0:
                #    self.label_pic.setPixmap(self.image_list[0])


        QImg2 = QImage(img2.data, self.width, self.height, self.bytesPerLine,QImage.Format_RGB888)

        qpix=QPixmap.fromImage(QImg2)
        self.label_pic.setPixmap(qpix)

    def imu_callback(self,imu_msg):
        (r, p, y) = tf.transformations.euler_from_quaternion( \
                    [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, \
                     imu_msg.orientation.w])
        """
        angle=y*180/3.1415926
        self.lcdNumber_imu.display(angle)
        """
        self.updata_imu.emit()

    def slotimu(self):
        angle=y*180/3.1415926
        self.lcdNumber_imu.display(angle)

    def arm_status_callback(self,status_msg):
        self.textEdit.setText(status_msg.data)

