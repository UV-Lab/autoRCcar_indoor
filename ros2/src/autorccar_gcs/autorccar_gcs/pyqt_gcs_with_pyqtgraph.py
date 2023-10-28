import sys, io, json, os
from tracemalloc import start
import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from threading import Thread
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from std_msgs.msg import Int8, Float32
from autorccar_interfaces.msg import NavState, Path, PathPoint
import array
import numpy as np
import math
import copy
from .submodules.user_geometry import *
from .submodules.cubic_spline import *


forceQuit = False
start_coord = [0, 0]
goal_coord = [0, 0]
flagNavUpdate = False
posE, posN, posU, velE, velN, velU = 0, 0, 0, 0, 0, 0
Roll, Pitch, Yaw = 0, 0, 0
oriLat, oriLon, oriHei = 37.540022, 127.076111, 0
i = 0
path_ned = []

flagPathPosUpdate = False


class Ros2Node(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.publisher_command = self.create_publisher(Int8, "gcs/command", 10)
        self.publisher_setyaw = self.create_publisher(Float32, "setyaw_topic", 10)
        self.publisher_global_path = self.create_publisher(Path, "gcs/global_path", 10)
        self.subscription_nav = self.create_subscription(
            NavState, "nav_topic", self.NavSubCallback, 10
        )

    def PublishCommand(self, command):
        msg = Int8()
        msg.data = command
        self.get_logger().info('pub-command: "%d"' % msg.data)
        self.publisher_command.publish(msg)

    def PublishSetYaw(self, yaw):
        msg = Float32()
        msg.data = yaw
        self.get_logger().info('pub-setYaw: "%lf"' % msg.data)
        self.publisher_setyaw.publish(msg)

    def PublishGlobalPath(self, path_list):
        msg = Path()
        for point in path_list:
            msg.path_points.append(PathPoint(x=point[0], y=point[1], speed=0.0))
        self.publisher_global_path.publish(msg)

    def NavSubCallback(self, msg):
        global posE, posN, posU, velN, velE, velU
        global Roll, Pitch, Yaw
        global oriLat, oriLon, oriHei
        global flagNavUpdate

        posE = msg.position.x
        posN = msg.position.y
        posU = msg.position.z

        velE = msg.velocity.x
        velN = msg.velocity.y
        velU = msg.velocity.z

        qx = msg.quaternion.x
        qy = msg.quaternion.y
        qz = msg.quaternion.z
        qw = msg.quaternion.w

        eulr = quat2eulr([qw, qx, qy, qz])

        Roll = eulr[0] * 180 / math.pi
        Pitch = eulr[1] * 180 / math.pi
        Yaw = eulr[2] * 180 / math.pi

        # llh_tmp = xyz2llh([msg.origin.x, msg.origin.y, msg.origin.z])

        # oriLat = llh_tmp[0] * 180 / math.pi
        # oriLon = llh_tmp[1] * 180 / math.pi
        # oriHei = llh_tmp[2]

        flagNavUpdate = True


def WindowStyle(app):
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)


class GCSWindow(QMainWindow):
    def __init__(self, ros2_node):
        super().__init__()
        self.setWindowTitle("GCS")
        self.window_width, self.window_height = 1028, 720
        self.setMinimumSize(self.window_width, self.window_height)
        self.initUI()
        self.ResetParameters()

        self.ros2_node = ros2_node
        self.ros2_thread = Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.ros2_thread.start()

        self.timer_0 = QTimer(self)
        self.timer_0.start(100)  # [ms]
        self.timer_0.timeout.connect(self.DataUpdateCheck)

    def initUI(self):
        tabs = QTabWidget()
        tab1 = QWidget()
        tab2 = QWidget()
        tab3 = QWidget()

        tabs.addTab(tab1, "main")
        tabs.addTab(tab2, "details")
        tabs.addTab(tab3, "param")

        self.setCentralWidget(tabs)

        grid_main = QGridLayout()

        tab1.setLayout(grid_main)

        self.position_graph = self.SetWidgetPositionGraph()
        self.position_graph.plotItem.setMenuEnabled(False)
        self.position_graph.scene().sigMouseClicked.connect(
            self.PositionGraphMouseClicedCallback
        )

        labelfont_1 = QLabel().font()
        labelfont_1.setPointSize(12)
        labelfont_1.setBold(True)

        grid_main_1 = QGridLayout()

        grid1_lab = QLabel("Graph")
        grid1_lab.setFont(labelfont_1)
        grid1_lab.setAlignment(Qt.AlignCenter)
        grid_main_1.addWidget(self.position_graph, 0, 0)
        grid_main_1.addWidget(QProgressBar(self), 1, 0)

        self.layout_pos = self.SetNavVlayout("Pos", ["E", "N", "U"])
        self.layout_vel = self.SetNavVlayout("Vel", ["E", "N", "U"])
        self.layout_att = self.SetNavVlayout("Att", ["Roll", "Pitch", "Yaw"])

        grid_main_2 = QGridLayout()

        grid2_lab = QLabel("Nav")
        grid2_lab.setFont(labelfont_1)
        grid2_lab.setAlignment(Qt.AlignCenter)

        grid_main_2_1 = QGridLayout()
        grid_main_2_2 = QGridLayout()

        grid_main_2_1.addLayout(self.layout_pos, 0, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_vel, 1, 0, alignment=Qt.AlignTop)
        grid_main_2_1.addLayout(self.layout_att, 2, 0, alignment=Qt.AlignTop)

        btn_clear = QPushButton("Clear All", self)
        btn_import_path = QPushButton("Import Path", self)
        btn_export_path = QPushButton("Export Path", self)
        btn_send_path = QPushButton("Send Path", self)
        btn_set_yaw = QPushButton("Set Yaw", self)
        btn_manual = QPushButton("Manual", self)
        btn_start = QPushButton("Start", self)
        btn_stop = QPushButton("Stop", self)

        btn_clear.clicked.connect(self.BtnClearAllCallback)
        btn_import_path.clicked.connect(self.ImportPathCallback)
        btn_export_path.clicked.connect(self.ExportPathCallback)
        btn_send_path.clicked.connect(self.BtnSendPathCallback)
        btn_set_yaw.clicked.connect(self.BtnSetYawCallback)
        btn_manual.clicked.connect(self.BtnManualCallback)
        btn_start.clicked.connect(self.BtnStartCallback)
        btn_stop.clicked.connect(self.BtnStopCallback)

        grid_main_2_2.addWidget(btn_clear, 1, 0)
        grid_main_2_2.addWidget(btn_import_path, 2, 0)
        grid_main_2_2.addWidget(btn_export_path, 3, 0)
        grid_main_2_2.addWidget(btn_send_path, 4, 0)
        grid_main_2_2.addWidget(btn_set_yaw, 5, 0)
        grid_main_2_2.addWidget(btn_manual, 6, 0)
        grid_main_2_2.addWidget(btn_start, 7, 0)
        grid_main_2_2.addWidget(btn_stop, 8, 0)

        grid_main_2.addLayout(grid_main_2_1, 0, 0)
        grid_main_2.addLayout(grid_main_2_2, 1, 0)
        grid_main_2.setRowStretch(0, 1)
        grid_main_2.setRowStretch(1, 1)

        grid_main.addWidget(grid1_lab, 0, 0)
        grid_main.addWidget(grid2_lab, 0, 1)
        grid_main.addLayout(grid_main_1, 1, 0)
        grid_main.addLayout(grid_main_2, 1, 1)
        grid_main.setColumnStretch(0, 7)
        grid_main.setColumnStretch(1, 3)

    def ResetParameters(self):
        self.origin_filename_ = "test.txt"
        self.plot_points_ = []
        self.x_points_ = []
        self.y_points_ = []
        self.num_points_ = 0
        self.is_point_moving_ = False
        self.close_point_idx_ = 0
        self.res_x_points_ = []
        self.res_y_points_ = []
        self.res_headings_ = []
        self.res_curvatures_ = []
        self.res_distances = []
        self.plot_spline_ = []
        self.pos_traj_e = []
        self.pos_traj_n = []
        self.plot_current_pos = self.position_graph.plot(
            symbol="o", symbolSize=20, symbolBrush="y", name="current_pos"
        )
        self.plot_pos_traj = self.position_graph.plot(
            pen=pg.mkPen(width=2, color="y"), name="pos"
        )

    def SetWidgetPositionGraph(self):
        graph = pg.PlotWidget()

        graph.setTitle("Position")
        graph.setLabel("left", "North[m]")
        graph.setLabel("bottom", "East[m]")
        graph.showGrid(x=True, y=True)
        graph.addLegend()

        graph.setRange(rect=None, xRange=(-10, 10), yRange=(-10, 10))

        return graph

    def SetNavVlayout(self, title, str):
        titlefont = QLabel().font()
        titlefont.setBold(True)

        title_lab = QLabel(title)
        title_lab.setFont(titlefont)

        layout = QGridLayout()
        layout.addWidget(title_lab, 0, 0)
        layout.addWidget(QLabel(str[0]), 1, 0)
        layout.addWidget(QLineEdit("0.0000"), 1, 1)
        layout.addWidget(QLabel(str[1]), 2, 0)
        layout.addWidget(QLineEdit("0.0000"), 2, 1)
        layout.addWidget(QLabel(str[2]), 3, 0)
        layout.addWidget(QLineEdit("0.0000"), 3, 1)

        return layout

    def PositionGraphMouseClicedCallback(self, evt):
        vb = self.position_graph.plotItem.vb
        scene_coords = evt.scenePos()
        if self.position_graph.sceneBoundingRect().contains(scene_coords):
            mouse_point = vb.mapSceneToView(scene_coords)
            if evt.button() == 1:
                if self.is_point_moving_:
                    close_point_idx = self.close_point_idx_
                    self.position_graph.removeItem(self.plot_points_[close_point_idx])
                    del self.plot_points_[close_point_idx]
                    plot_point = self.position_graph.plot(
                        x=[mouse_point.x()],
                        y=[mouse_point.y()],
                        symbol="o",
                        symbolSize=20,
                        symbolBrush="w",
                    )
                    self.plot_points_.insert(close_point_idx, plot_point)
                    self.x_points_[close_point_idx] = mouse_point.x()
                    self.y_points_[close_point_idx] = mouse_point.y()
                    self.is_point_moving_ = False
                    self.UpdatePath()
                else:
                    close_point_idx = self.IsClosePoint(mouse_point)
                    if close_point_idx != []:
                        self.position_graph.removeItem(
                            self.plot_points_[close_point_idx]
                        )
                        del self.plot_points_[close_point_idx]
                        plot_point = self.position_graph.plot(
                            x=[self.x_points_[close_point_idx]],
                            y=[self.y_points_[close_point_idx]],
                            symbol="o",
                            symbolSize=20,
                            symbolBrush="r",
                        )
                        self.plot_points_.insert(close_point_idx, plot_point)
                        self.is_point_moving_ = True
                        self.close_point_idx_ = close_point_idx
                    else:
                        print("add point")
                        self.x_points_.append(mouse_point.x())
                        self.y_points_.append(mouse_point.y())
                        plot_point = self.position_graph.plot(
                            x=[mouse_point.x()],
                            y=[mouse_point.y()],
                            symbol="o",
                            symbolSize=20,
                            symbolBrush="w",
                        )
                        self.plot_points_.append(plot_point)
                        self.num_points_ += 1
                        self.UpdatePath()
            elif evt.button() == 2:
                self.position_graph.removeItem(self.plot_points_[-1])
                del self.plot_points_[-1]
                del self.x_points_[-1]
                del self.y_points_[-1]
                self.num_points_ -= 1
                self.UpdatePath()

    def UpdatePath(self):
        if self.num_points_ > 1:
            [
                self.res_x_points_,
                self.res_y_points_,
                self.res_headings_,
                self.res_curvatures_,
                self.res_distances,
            ] = CalculateCubicSplinePath(self.x_points_, self.y_points_, 0.5)
            self.position_graph.removeItem(self.plot_spline_)
            self.plot_spline_ = self.position_graph.plot(
                x=self.res_x_points_,
                y=self.res_y_points_,
                pen=pg.mkPen(width=2, color="w", style=QtCore.Qt.DashLine),
            )

            self.pos_traj_e = []
            self.pos_traj_n = []

    def IsClosePoint(self, mouse_point):
        for idx in range(self.num_points_):
            d = math.sqrt(
                (mouse_point.x() - self.x_points_[idx]) ** 2
                + (mouse_point.y() - self.y_points_[idx]) ** 2
            )
            if d < 0.1:
                return idx
        return []

    def ClearPlotPoints(self):
        self.plot_points_ = []
        self.position_graph.clear()

    def BtnClearAllCallback(self):
        self.ClearPlotPoints()
        self.ResetParameters()
        self.position_graph.setRange(rect=None, xRange=(-10, 10), yRange=(-10, 10))

    def BtnSendPathCallback(self):
        path_list = []
        for idx in range(len(self.x_points_)):
            path_list.append([self.x_points_[idx], self.y_points_[idx]])

        print("send path")
        self.ros2_node.PublishGlobalPath(path_list)

    def BtnSetYawCallback(self):
        val, ok = QInputDialog.getDouble(
            self, "Set Yaw", "Enter the yaw angle relative to true north:"
        )
        if ok:
            yaw = val
            while yaw > 180:
                yaw = yaw - 360
            while yaw <= -180:
                yaw = yaw + 360
            print("set yaw")
            self.ros2_node.PublishSetYaw(yaw)

    def BtnManualCallback(self):
        print("manual")
        self.ros2_node.PublishCommand(2)

    def BtnStartCallback(self):
        print("start")
        self.ros2_node.PublishCommand(1)

    def BtnStopCallback(self):
        print("stop")
        self.ros2_node.PublishCommand(0)

    def DataUpdateCheck(self):
        global flagNavUpdate, posE, posN, posU, velE, velN, velU
        global Roll, Pitch, Yaw
        global oriLat, oriLon, oriHei

        if flagNavUpdate:
            self.layout_pos.itemAt(2).widget().setText(str(posE))
            self.layout_pos.itemAt(4).widget().setText(str(posN))
            self.layout_pos.itemAt(6).widget().setText(str(posU))

            self.layout_vel.itemAt(2).widget().setText(str(velE))
            self.layout_vel.itemAt(4).widget().setText(str(velN))
            self.layout_vel.itemAt(6).widget().setText(str(velU))

            self.layout_att.itemAt(2).widget().setText(str(Roll))
            self.layout_att.itemAt(4).widget().setText(str(Pitch))
            self.layout_att.itemAt(6).widget().setText(str(Yaw))

            self.UpdateGraphWithCurrentPos()

            flagNavUpdate = False

    def UpdateGraphWithCurrentPos(self):
        global posE, posN
        self.pos_traj_e.append(posE)
        self.pos_traj_n.append(posN)
        self.plot_current_pos.setData(x=[posE], y=[posN])
        self.plot_pos_traj.setData(x=self.pos_traj_e, y=self.pos_traj_n)

    def ImportPathCallback(self):
        filename = QtWidgets.QFileDialog.getOpenFileName(self, "Import Path File")
        self.origin_filename_ = filename[0]
        with open(self.origin_filename_, "r") as f:
            lines = f.readlines()
            self.x_points_ = []
            self.y_points_ = []
            self.ClearPlotPoints()
            for line in lines:
                line.strip()
                splited = line.split()
                listfloat = list(map(float, splited))
                self.x_points_.append(listfloat[0])
                self.y_points_.append(listfloat[1])
                plot_point = self.position_graph.plot(
                    x=[listfloat[0]],
                    y=[listfloat[1]],
                    symbol="o",
                    symbolSize=20,
                    symbolBrush="y",
                )
                self.plot_points_.append(plot_point)
            self.num_points_ = len(self.x_points_)
            x_max = max(self.x_points_)
            x_min = min(self.x_points_)
            x_cen = (x_max + x_min) / 2.0
            y_max = max(self.y_points_)
            y_min = min(self.y_points_)
            y_cen = (y_max + y_min) / 2.0
            x_half_range = (x_max - x_min) / 2.0 + 2
            y_half_range = (y_max - y_min) / 2.0 + 2
            if x_half_range > y_half_range:
                x_range = (x_cen - x_half_range, x_cen + x_half_range)
                y_range = (y_cen - x_half_range, y_cen + x_half_range)
            else:
                x_range = (x_cen - y_half_range, x_cen + y_half_range)
                y_range = (y_cen - y_half_range, y_cen + y_half_range)
            self.position_graph.setRange(rect=None, xRange=x_range, yRange=y_range)
        self.UpdatePath()
        print("path is imported")

    def ExportPathCallback(self):
        filename = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Path File",
            os.path.join("", self.origin_filename_),
            "Text files (*.txt)",
        )
        filename_text = filename[0]
        export_list = []
        for idx in range(len(self.x_points_)):
            export_list.append([self.x_points_[idx], self.y_points_[idx]])

        with open(filename_text, "w") as file:
            file.writelines(" ".join(str(j) for j in i) + "\n" for i in export_list)
        print("points are exported")

    def closeEvent(self, event):
        print("Close window")
        rclpy.shutdown()
        self.ros2_thread.join()
        global forceQuit
        forceQuit = True


def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    WindowStyle(app)

    rclpy.init(args=args)

    node = Ros2Node("pyqt_gcs")

    gcsapp = GCSWindow(node)
    gcsapp.show()

    try:
        app.exec_()  # 이벤트 루프 시작
    except forceQuit:
        app.quit()


if __name__ == "__main__":
    main()
