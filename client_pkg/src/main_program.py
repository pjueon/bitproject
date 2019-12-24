#!/usr/bin/env python3
import sys
import cv2
import rospy
import numpy as np
import os
import time

from darkflow.net.build import TFNet

from std_msgs.msg import Bool, String

from PySide2.QtWidgets import (QLineEdit, QPushButton, QApplication, QAction,
    QVBoxLayout, QDialog, QMainWindow, QGraphicsScene, QGraphicsPixmapItem)
from PySide2.QtGui import QPixmap, QImage, QKeySequence, QColor
from PySide2.QtCore import QThread, Signal, Slot, QEvent

from main_ui import Ui_Form

from py_pkg import (Create_Map, camera_thread, map_reader_thread,
                    srv_thread, load_thread, book_search_thread)

def fixpath(path):
    return os.path.abspath(os.path.expanduser(path))

class MainWindow(QMainWindow):
#////////////////////////// MainWindow Init Start //////////////////////////////
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui.Form.installEventFilter(self)
        self.ui.statusbar.showMessage('Ready', 5000)
        self.camera_flag = 0
        self.loading_flag = True
        self.map_load_flag = False
        self.map_create_flag = False

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

        self.th_srv = srv_thread.Server(parent = self)

        self.th_load = load_thread.Loader(parent = self)

        self.th_book = book_search_thread.Book_search(parent = self)

#========================== Publisher Define ===================================
        self.camera_pub = rospy.Publisher("/camera_toggle",
                                          Bool,
                                          queue_size = 1)

        self.launch_select_pub = rospy.Publisher("/launch_select",
                                          String,
                                          queue_size = 1)

#========================== Etc Class init =====================================
        self.CM = Create_Map.create_map()

#========================== Show Mainwindow ====================================
        self.show()

#////////////////////////// MainWindow Init End ////////////////////////////////


#========================== Auto Control Slot Def ==============================
    @Slot()
    def auto_Start(self):
        try:
            print(self.ui.listwidget.count())
            if(not self.ui.listwidget.count() == 0):
                self.th_book.sub.unregister()
                self.th_book.quit()
                self.ui.listwidget.clear()

            print(self.ui.listwidget.count())
            self.ui.statusbar.showMessage('Auto Drive Start')
            self.launch_select_pub.publish("auto_Start")
            self.th_book.start()
            self.ui.Map_View.setFocus()
        except:
            pass
    @Slot()
    def auto_Stop(self):
        try:

            self.ui.statusbar.showMessage('Auto Drive End', 5000)
            self.launch_select_pub.publish("auto_Stop")

        except:
            pass

#========================== Camera Control Slot Def ============================
    @Slot()
    def Camera_Toggle_BTN(self):
        try:
            if (self.camera_flag == 0):
                self.camera_on()
            else:
                self.camera_off()
        except:
            pass

#========================== Map Control Slot Def ===============================
    @Slot()
    def read_Map(self):
        if(self.map_load_flag == False):
            self.map_load_flag = True
            self.th_load.loading_flag = True

            self.ui.Map_View.installEventFilter(self)
            self.ui.Map_View.setFocus()

            self.th_load.loading.connect(self.loading_map)
            self.th_map.send_map_view.connect(self.map_View_Update)
            self.launch_select_pub.publish("load_map_mode")

            self.th_load.start()
            self.th_map.start()

    @Slot()
    def delete_Map(self):
        if(self.map_load_flag == True):
            self.th_map.send_map_view.disconnect()
        self.map_load_flag = False
        self.map_create_flag = False
        self.th_load.loading_flag = False

        self.ui.Map_View.removeEventFilter(self)

        self.th_map.stop()
        self.th_map.quit()

        self.th_load.quit()

        time.sleep(0.1)
        self.launch_select_pub.publish("load_map_mode_close")
        self.ui.Map_View.setScene(self.view_Clear())
        self.ui.Map_View.show()

        self.camera_off()

    @Slot()
    def create_Map(self):
        if(self.map_create_flag == False):
            self.map_create_flag = True
            self.th_load.loading_flag = True

            self.th_load.loading.connect(self.loading_map)
            self.th_map.send_map_view.connect(self.map_View_Update)
            self.launch_select_pub.publish("create_map_mode")
            self.ui.Map_View.installEventFilter(self)

            self.th_load.start()
            self.th_map.start()

    @Slot()
    def save_Map(self):
        if(self.map_create_flag == True):
            self.th_map.stop()
            self.th_map.quit()

            self.ui.Map_View.removeEventFilter(self)

            self.launch_select_pub.publish("load_map_mode_close")
            self.ui.Map_View.setScene(self.view_Clear())
            self.ui.Map_View.show()

            self.th_load.loading_flag = False
            self.th_srv.send_server_data.connect(self.srv_Server)
            self.th_srv.start()
            self.camera_off()

#========================== Thread Data Req, Res Slot Def ======================
    @Slot(object)
    def map_View_Update(self, msg):
        if (self.th_load.loading_flag == True):

            self.th_load.loading_flag = False
            self.th_load.loading.disconnect()
            self.camera_on()
        try:
            scene = self.image2Qpixmap(msg)
            self.ui.Map_View.setScene(scene)
            self.ui.Map_View.show()
        except:
            pass

    @Slot(object)
    def camera_View_Update(self, msg):
        try:
            scene = self.image2Qpixmap(msg)
            self.ui.Camera_View.setScene(scene)
            self.ui.Camera_View.show()
        except:
            pass

    @Slot(object)
    def srv_Server(self):
        try:
            self.map_create_flag = False
            self.launch_select_pub.publish("create_map_mode_save")
            self.th_map.send_map_view.disconnect()
            self.th_map.stop()
            self.th_map.quit()
            self.ui.Map_View.setScene(self.view_Clear())
            self.ui.Map_View.show()
        except:
            pass

    @Slot(object)
    def loading_map(self, msg):
        try:
            scene = self.image2Qpixmap(msg)
            self.ui.Map_View.setScene(scene)
            self.ui.Map_View.show()
        except:
            pass
#========================== Event Filter =======================================
    def eventFilter(self, obj, event):
        if event.type() == QEvent.Close:
            print ("================ User has clicked the red x on the main window =================\n\n\n")

            #del self.CM
            if(self.th_load.isRunning()):
                self.th_load.loading_flag = False
                self.th_load.quit()
                self.th_load.wait(2)
            self.launch_select_pub.publish("auto_Stop")
            self.launch_select_pub.publish("load_map_mode_close")

            event.accept()
            print("============================= End Mainwindow ===================================\n\n\n")

            return True
        elif obj == self.ui.Map_View:
            if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
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

#    def closeEvent(self, event): # Click the Window X Button, Calling this Event

#========================== Etc Function =======================================

    def view_Clear(self):
        try:
            clear = QPixmap(1, 1)
            clear.fill(QColor("white"))
            clear = QGraphicsPixmapItem(clear)
            scene = QGraphicsScene()
            scene.addItem(clear)
            return scene
        except:
            pass

    def image2Qpixmap(self, data):
        pixmap = QPixmap.fromImage(data)
        item = QGraphicsPixmapItem(pixmap)
        scene = QGraphicsScene()
        scene.addItem(item)
        return scene

    def camera_on(self):
        if (self.camera_flag == 0):
            self.ui.statusbar.showMessage('Camera On')
            self.th_camera.send_camera_view.connect(self.camera_View_Update)
            self.th_camera.start()
            self.camera_flag = 1
            self.camera_pub.publish(self.camera_flag)
            self.ui.Camera_Toggle_BTN.setText("카메라 Off")
            #print("카메라 On")

    def camera_off(self):
        if (self.camera_flag == 1):
            self.ui.statusbar.showMessage('Camera Off', 5000)

            self.th_camera.send_camera_view.disconnect()
            self.th_camera.stop()
            self.camera_flag = 0
            self.camera_pub.publish(self.camera_flag)
            self.ui.Camera_Toggle_BTN.setText("카메라 On")
            #print("카메라 Off")
            time.sleep(0.1)
            self.ui.Camera_View.setScene(self.view_Clear())
            self.ui.Camera_View.show()


if __name__ == '__main__':

    options = {"pbLoad" : fixpath("~/catkin_ws/src/bitproject/built_graph/book_2class_yolo2.pb"),
               "metaLoad" : fixpath("~/catkin_ws/src/bitproject/built_graph/book_2class_yolo2.meta"),
               "threshold" : 0.5
               }

    tfnet = TFNet(options)
    rospy.init_node('main_program', anonymous=True)
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
