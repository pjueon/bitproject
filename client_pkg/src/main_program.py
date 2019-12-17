#!/usr/bin/env python3
import sys
import cv2
import rospy
import numpy as np
import os

from darkflow.net.build import TFNet

from std_msgs.msg import Bool, String

from PySide2.QtWidgets import (QLineEdit, QPushButton, QApplication, QAction,
    QVBoxLayout, QDialog, QMainWindow, QGraphicsScene, QGraphicsPixmapItem)
from PySide2.QtGui import QPixmap, QImage, QKeySequence, QColor
from PySide2.QtCore import QThread, Signal, Slot, QEvent

from main_ui import Ui_Form

from py_pkg import Create_Map
from py_pkg import camera_thread
from py_pkg import map_reader_thread
from py_pkg import srv_thread

def fixpath(path):
    return os.path.abspath(os.path.expanduser(path))

class MainWindow(QMainWindow):
    camera_flag = 0
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui.Form.installEventFilter(self)
        self.ui.statusbar.showMessage('Ready')

#========================== Auto Control Button init ===========================
        self.ui.Auto_Fineder_Start_BTN.clicked.connect(self.auto_Start)
        self.ui.Auto_Fineder_Stop_BTN.clicked.connect(self.auto_Stop)

#========================== Map Control Button init ============================
        self.ui.Map_Reader_BTN.clicked.connect(self.read_Map)
        self.ui.Map_Delete_BTN.clicked.connect(self.delete_Map)
        self.ui.Map_Save_BTN.clicked.connect(self.save_Map)
        self.ui.Map_Create_BTN.clicked.connect(self.create_Map)

#========================== Camera Control Button init =========================
        self.ui.Camera_Toggle_BTN.clicked.connect(self.Camera_Toggle_BTN)

#========================== Thread init ========================================
        self.th_camera = camera_thread.Worker(parent = self, tfnet = tfnet)
        self.th_camera.send_camera_view.connect(self.camera_View_Update)

        self.th_map = map_reader_thread.Mapper(parent = self)
        self.th_map.send_map_view.connect(self.map_View_Update)

        #self.th_srv = srv_thread.Server(parent=self)

#========================== Publisher init =====================================
        self.camera_pub = rospy.Publisher("/camera_toggle", Bool, queue_size = 1)
        self.node_pub = rospy.Publisher("/node_pub", String, queue_size = 1)
        self.launch_select_pub = rospy.Publisher("/launch_select", String, queue_size = 1)
        #self.th_srv.start()

        self.CM = Create_Map.create_map()

        self.show()

#========================== Auto Control Slot Def ==============================
    @Slot()
    def auto_Start(self):
        try:
            self.node_pub.publish("auto_Start")
        except:
            pass
    @Slot()
    def auto_Stop(self):
        try:
            self.node_pub.publish("auto_Stop")
        except:
            pass

#========================== Camera Control Slot Def ============================
    @Slot()
    def Camera_Toggle_BTN(self):
        try:
            if (self.camera_flag == 0):
                self.th_camera.start()
                self.camera_flag = 1
                self.ui.Camera_Toggle_BTN.setText("카메라 Off")
                print("카메라 On")
            else:
                self.th_camera.quit()
                self.camera_flag = 0
                self.ui.Camera_Toggle_BTN.setText("카메라 On")
                print("카메라 Off")
            self.camera_pub.publish(self.camera_flag)
        except:
            pass

    @Slot(object)
    def camera_View_Update(self, msg):
        try:
            item = QGraphicsPixmapItem(msg)
            scene = QGraphicsScene()
            scene.addItem(item)
            self.ui.Camera_View.setScene(scene)
            self.ui.Camera_View.show()
        except:
            pass

#========================== Map Control Slot Def ===============================
    @Slot()
    def read_Map(self):
        self.launch_select_pub.publish("load_map_mode")
        self.th_map.start()
        #self.th_srv.add_trigger_init_client()
        #self.node_pub.publish("auto_start")

    @Slot()
    def delete_Map(self):

        clear = QPixmap(500, 500)
        clear.fill(QColor("white"))
        clear = QGraphicsPixmapItem(clear)
        self.scene = QGraphicsScene()
        self.scene.addItem(clear)
        self.ui.Map_View.setScene(self.scene)
        self.ui.Map_View.show()

        self.launch_select_pub.publish("load_map_mode_close")
        self.th_map.quit()

    @Slot()
    def create_Map(self):
        self.launch_select_pub.publish("create_map_mode")
        self.ui.Map_View.installEventFilter(self)
        self.ui.Map_View.setFocus()

    @Slot()
    def save_Map(self):
        self.launch_select_pub.publish("create_map_mode_save")

    @Slot(object)
    def map_View_Update(self, msg):
        try:
            item = QGraphicsPixmapItem(msg)
            scene = QGraphicsScene()
            scene.addItem(item)
            self.ui.Map_View.setScene(scene)
            self.ui.Map_View.show()
        except:
            pass

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Close:
            print ("================ User has clicked the red x on the main window =================\n\n\n")

            del self.CM
            try:
                self.th_camera.quit()
                self.th_map.quit()
                del self.th_camera
                del self.th_map
                event.accept()

                #if self.th_srv.isRunning():
                #    self.th_srv.quit()
            except:
                del self.th_camera
                del self.th_map

            event.accept()


            print("============================= End Mainwindow ===================================\n\n\n")

            return True
        elif obj == self.ui.Map_View:
            if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
                print(event.key())
                self.CM.keyEvent(event)
                return True
            elif event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
                self.CM.keyEvent(event)
                return True
            else:
                return False
        else:
            # pass the event on to the parent class
            return QMainWindow.eventFilter(self, obj, event)

#    def closeEvent(self, event): #X버튼 누르면 실행되는 이벤트
if __name__ == '__main__':

    options = {"pbLoad" : fixpath("~/catkin_ws/src/bitproject/built_graph/book_3class_yolo2.pb"),
               "metaLoad" : fixpath("~/catkin_ws/src/bitproject/built_graph/book_3class_yolo2.meta"),
               "threshold" : 0.1
               }

    tfnet = TFNet(options)
    rospy.init_node('main_program')
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
