# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'exam1.ui',
# licensing of 'main_ui.ui' applies.
#
# Created: Tue Dec 10 09:14:44 2019
#      by: pyside2-uic  running on PySide2 5.13.2
#
# WARNING! All changes made in this file will be lost!

from PySide2 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        self.Form = Form
        self.Form.setObjectName("Bit Client")
        self.Form.resize(1855, 1020)
        self.Form.setMinimumSize(QtCore.QSize(1855, 1020))
        self.Form.setMaximumSize(QtCore.QSize(1855, 1020))
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.Form)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 1850, 1015))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.main_Grid = QtWidgets.QGridLayout(self.horizontalLayoutWidget)
        self.main_Grid.setObjectName("main_Grid")
        #self.main_Grid.setContentsMargins(5, 5, 5, 5)
        #self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        #self.horizontalLayout.setContentsMargins(5, 5, 5, 5)
        #self.horizontalLayout.setObjectName("horizontalLayout")
        self.Map_View = QtWidgets.QGraphicsView(self.horizontalLayoutWidget)
        self.Map_View.setMinimumSize(QtCore.QSize(932, 932))
        self.Map_View.setMaximumSize(QtCore.QSize(932, 932))
        self.Map_View.setObjectName("Map_View")
        #self.horizontalLayout.addWidget(self.Map_View)
        self.main_Grid.addWidget(self.Map_View, 1, 0, 1, 1)

        self.Map_view_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Map_view_Label.setMinimumSize(QtCore.QSize(150, 20))
        self.Map_view_Label.setMaximumSize(QtCore.QSize(150, 20))
        self.Map_view_Label.setAlignment(QtCore.Qt.AlignLeft)
        self.Map_view_Label.setObjectName("Map_view_Label")
        self.main_Grid.addWidget(self.Map_view_Label, 0, 0, 1, 1)

        self.Camera_view_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Camera_view_Label.setMinimumSize(QtCore.QSize(150, 20))
        self.Camera_view_Label.setMaximumSize(QtCore.QSize(150, 20))
        self.Camera_view_Label.setAlignment(QtCore.Qt.AlignLeft)
        self.Camera_view_Label.setObjectName("Camera_view_Label")
        self.main_Grid.addWidget(self.Camera_view_Label, 0, 1, 1, 1)


        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.button_Grid = QtWidgets.QGridLayout()
        self.button_Grid.setObjectName("button_Grid")
        self.camera_Grid = QtWidgets.QGridLayout()
        self.camera_Grid.setObjectName("camera_Grid")

        spacerItem = QtWidgets.QSpacerItem(115, 30, QtWidgets.QSizePolicy.Maximum)
        spacerItem1 = QtWidgets.QSpacerItem(115, 30, QtWidgets.QSizePolicy.Maximum)
        self.button_Grid.addItem(spacerItem, 0, 1, 1, 1)
        self.button_Grid.addItem(spacerItem1, 0, 4, 1, 1)
        self.Auto_Control_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Auto_Control_Label.setMinimumSize(QtCore.QSize(150, 20))
        self.Auto_Control_Label.setMaximumSize(QtCore.QSize(150, 20))
        self.Auto_Control_Label.setAlignment(QtCore.Qt.AlignCenter)
        self.Auto_Control_Label.setObjectName("Auto_Control_Label")
        self.button_Grid.addWidget(self.Auto_Control_Label, 0, 0, 1, 1)

        self.Map_Control_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Map_Control_Label.setMinimumSize(QtCore.QSize(300, 20))
        self.Map_Control_Label.setMaximumSize(QtCore.QSize(300, 20))
        self.Map_Control_Label.setAlignment(QtCore.Qt.AlignCenter)
        self.Map_Control_Label.setObjectName("Map_Control_Label")
        self.button_Grid.addWidget(self.Map_Control_Label, 0, 2, 1, -1)

        self.Camera_Control_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Camera_Control_Label.setMinimumSize(QtCore.QSize(150, 20))
        self.Camera_Control_Label.setMaximumSize(QtCore.QSize(150, 20))
        self.Camera_Control_Label.setAlignment(QtCore.Qt.AlignCenter)
        self.Camera_Control_Label.setObjectName("Camera_Control_Label")
        self.button_Grid.addWidget(self.Camera_Control_Label, 0, 5, 1, 1)

        self.Result_Control_Label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.Result_Control_Label.setMinimumSize(QtCore.QSize(150, 20))
        self.Result_Control_Label.setMaximumSize(QtCore.QSize(150, 20))
        self.Result_Control_Label.setAlignment(QtCore.Qt.AlignCenter)
        self.Result_Control_Label.setObjectName("Result_Control_Label")
        self.button_Grid.addWidget(self.Result_Control_Label, 4, 0, 1, 1)

        self.Auto_Fineder_Start_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Auto_Fineder_Start_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Auto_Fineder_Start_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Auto_Fineder_Start_BTN.setObjectName("Auto_Fineder_Start_BTN")
        self.button_Grid.addWidget(self.Auto_Fineder_Start_BTN, 1, 0, 1, 1)

        self.Auto_Fineder_Stop_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Auto_Fineder_Stop_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Auto_Fineder_Stop_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Auto_Fineder_Stop_BTN.setObjectName("Auto_Fineder_Stop_BTN")
        self.button_Grid.addWidget(self.Auto_Fineder_Stop_BTN, 2, 0, 1, 1)

        self.Camera_Toggle_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Camera_Toggle_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Camera_Toggle_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Camera_Toggle_BTN.setObjectName("Camera_Toggle_BTN")
        self.button_Grid.addWidget(self.Camera_Toggle_BTN, 1, 5, 1, 1)

        self.Map_Reader_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Map_Reader_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Map_Reader_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Map_Reader_BTN.setObjectName("Map_Reader_BTN")
        self.button_Grid.addWidget(self.Map_Reader_BTN, 1, 2, 1, 1)

        self.Map_Delete_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Map_Delete_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Map_Delete_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Map_Delete_BTN.setObjectName("Map_Delete_BTN")
        self.button_Grid.addWidget(self.Map_Delete_BTN, 2, 2, 1, 1)

        self.Map_Save_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Map_Save_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Map_Save_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Map_Save_BTN.setObjectName("Map_Save_BTN")
        self.button_Grid.addWidget(self.Map_Save_BTN, 2, 3, 1, 1)

        self.Map_Create_BTN = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.Map_Create_BTN.setMinimumSize(QtCore.QSize(150, 30))
        self.Map_Create_BTN.setMaximumSize(QtCore.QSize(150, 30))
        self.Map_Create_BTN.setObjectName("Map_Create_BTN")
        self.button_Grid.addWidget(self.Map_Create_BTN, 1, 3, 1, 1)

        self.listwidget = QtWidgets.QListWidget()
        self.listwidget.setMinimumSize(QtCore.QSize(895, 315))
        self.listwidget.setMaximumSize(QtCore.QSize(895, 315))
        self.button_Grid.addWidget(self.listwidget, 5, 0, -1, -1)

        self.Camera_View = QtWidgets.QGraphicsView(self.horizontalLayoutWidget)
        self.Camera_View.setMinimumSize(QtCore.QSize(895, 475))
        self.Camera_View.setMaximumSize(QtCore.QSize(895, 475))
        self.Camera_View.setObjectName("Camera_View")
        self.camera_Grid.addWidget(self.Camera_View, 0, 0, 1, 1)

        self.camera_Grid.addLayout(self.button_Grid, 1, 0, 1, 1)

        self.horizontalLayout.addLayout(self.camera_Grid)
        #self.horizontalLayout.addLayout(self.horizontalLayout)
        self.main_Grid.addLayout(self.horizontalLayout, 1, 1, 1, 1)

        self.statusbar = QtWidgets.QStatusBar(self.Form)
        self.statusbar.setObjectName("statusbar")
        Form.setStatusBar(self.statusbar)

        self.retranslateUi(self.Form)
        QtCore.QMetaObject.connectSlotsByName(self.Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtWidgets.QApplication.translate("Form", "Bit Client", None, -1))
        self.Auto_Control_Label.setText(QtWidgets.QApplication.translate("Form", "Auto Control", None, -1))
        self.Camera_Control_Label.setText(QtWidgets.QApplication.translate("Form", "Camera Control", None, -1))
        self.Map_Control_Label.setText(QtWidgets.QApplication.translate("Form", "Map", None, -1))
        self.Result_Control_Label.setText(QtWidgets.QApplication.translate("Form", "Result", None, -1))
        self.Map_view_Label.setText(QtWidgets.QApplication.translate("Form", "Map View", None, -1))
        self.Camera_view_Label.setText(QtWidgets.QApplication.translate("Form", "Camera View", None, -1))
        self.Auto_Fineder_Start_BTN.setText(QtWidgets.QApplication.translate("Form", "시작", None, -1))
        self.Auto_Fineder_Stop_BTN.setText(QtWidgets.QApplication.translate("Form", "중지", None, -1))
        self.Camera_Toggle_BTN.setText(QtWidgets.QApplication.translate("Form", "카메라 On", None, -1))
        self.Map_Reader_BTN.setText(QtWidgets.QApplication.translate("Form", "맵 불러오기", None, -1))
        self.Map_Delete_BTN.setText(QtWidgets.QApplication.translate("Form", "맵 지우기", None, -1))
        self.Map_Save_BTN.setText(QtWidgets.QApplication.translate("Form", "맵 저장", None, -1))
        self.Map_Create_BTN.setText(QtWidgets.QApplication.translate("Form", "맵 만들기", None, -1))
