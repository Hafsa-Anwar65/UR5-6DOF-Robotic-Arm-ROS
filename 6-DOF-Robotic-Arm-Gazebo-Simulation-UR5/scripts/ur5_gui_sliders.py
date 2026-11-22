#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from PyQt5 import QtWidgets, QtCore


class UR5GUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR5 Joint Control GUI")
        self.setGeometry(300, 200, 500, 400)
        layout = QtWidgets.QVBoxLayout()

        # Joint names
        self.joint_names = [
            "shoulder_joint",
            "upper_arm_joint",
            "forearm_joint",
            "wrist1_joint",
            "wrist2_joint",
            "wrist3_joint",
        ]

        # ROS publishers
        self.pubs = {
            joint: rospy.Publisher(f"/{joint}_position_controller/command", Float64, queue_size=10)
            for joint in self.joint_names
        }

        # Sliders
        self.sliders = {}
        for joint in self.joint_names:
            label = QtWidgets.QLabel(joint)
            label.setAlignment(QtCore.Qt.AlignCenter)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(-157)
            slider.setMaximum(157)
            slider.setValue(0)
            slider.setSingleStep(1)
            slider.valueChanged.connect(lambda val, j=joint: self.update_joint(j, val / 100.0))
            layout.addWidget(label)
            layout.addWidget(slider)
            self.sliders[joint] = slider

        self.setLayout(layout)

    def update_joint(self, joint_name, rad_value):
        msg = Float64()
        msg.data = rad_value
        self.pubs[joint_name].publish(msg)


if __name__ == "__main__":
    rospy.init_node("ur5_gui_slider_controller", anonymous=True, disable_signals=True)
    app = QtWidgets.QApplication(sys.argv)
    gui = UR5GUI()
    gui.show()
    sys.exit(app.exec_())
