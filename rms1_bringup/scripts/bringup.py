#!/usr/bin/python3
# coding=UTF-8
# https://robomaster-dev.readthedocs.io/zh_CN/latest/text_sdk/apis.html
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import tf
import yaml
import socket
import threading
import time
import numpy as np
import h264decoder
from PIL import Image as PImage
import cv2
import math

rndis = '192.168.42.2'
wifi = '192.168.2.1'

class S1_ROS:
    def __init__(self, connect_ip = wifi):
        self.connect_ip = connect_ip
        robot_ctrl_host = (connect_ip,40923)
        robot_event_host = (connect_ip, 40925)
        robot_data_host = ('', 40924)

        self.ctrl_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.event_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.stream_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.audio_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.ctrl_socket.connect(robot_ctrl_host)
        self.event_socket.connect(robot_event_host)
        self.data_socket.bind(robot_data_host)

        self.global_frame = None
        self.decoder = h264decoder.H264Decoder()

        self.ctrl_thread = threading.Thread(target=self.ctrl_recv)
        self.event_thread = threading.Thread(target=self.event_recv)
        self.data_thread = threading.Thread(target=self.data_recv)
        self.stream_thread = threading.Thread(target=self.stream_recv)
        self.audio_thread = threading.Thread(target=self.audio_recv)

        self.axes = (-0.0, -0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.0)
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.calib_vel = [0.0, 0.0, 0.0]
        self.calib_vel_lastframe = [0.0, 0.0, 0.0]
        self.item_position = [0.0, 0.0, 0.0]
        self.euler_rpy = [0.0, 0.0, 0.0]
        self.pos_yaw = 0.0
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.item_pose_sub = rospy.Subscriber("/item_pose", PoseStamped, self.item_cali_pos_callback)
        self.item_vel_sub = rospy.Subscriber("/calib_vel", Twist, self.item_calib_vel_callback)
        self.image_pub = rospy.Publisher("/s1_image", Image, queue_size=10)
        self.camInfo_pub = rospy.Publisher("/s1_camInfo", CameraInfo, queue_size=10)
        self.bridge = CvBridge()
        self.br = tf.TransformBroadcaster()

        # FSM flag
        self.statusNum = 0
        self.planSuccess = False  # default false
        self.planStart = True
        self.scanStart = True  # default false
        self.scanSuccess = True
        self.calibSuccess = False  # default false
        self.gripSuccess = False
        self.releaseSuccess = False
        self.autoGrip = False
        self.manualContor = True

    # 相机信息打包
    def make_camera_msg(self):
        camera_info_msg = CameraInfo()
        cam_file = open('/home/tianbot/rm_ws/src/camera_info/head_camera.yaml')
        cam_data = yaml.safe_load(cam_file)

        camera_info_msg.header = rospy.Header()
        camera_info_msg.header.frame_id = cam_data["camera_name"]
        # camera_info_msg.header.stamp = rospy.Time.now()

        camera_info_msg.distortion_model = cam_data["distortion_model"]
        camera_info_msg.width = cam_data['image_width']
        camera_info_msg.height = cam_data['image_height']
        camera_info_msg.binning_x = 0
        camera_info_msg.binning_y = 0

        camera_info_msg.K = cam_data['camera_matrix']['data']
        camera_info_msg.D = cam_data['distortion_coefficients']['data']
        camera_info_msg.R = cam_data['rectification_matrix']['data']
        camera_info_msg.P = cam_data['projection_matrix']['data']

        return camera_info_msg

    # 启动线程，使能SDK
    def start(self):
        self.ctrl_thread.start()
        self.event_thread.start()
        ret = self.command('command')
        if ret:
            print('S1 SDK turn on!')
        self.command('robot mode chassis_lead')

    # 视频流开启
    def video_on(self):
        ret = self.command('stream on')
        if ret:
            print('S1 stream turn on!')
        robot_stream_host = (self.connect_ip, 40921)
        self.stream_socket.connect(robot_stream_host)
        self.stream_thread.start()

    # 音频流开启
    def audio_on(self):
        ret = self.command('audio on')
        if ret:
            print('S1 audio turn on!')
        robot_audio_host = (self.connect_ip, 40922)
        self.audio_socket.connect(robot_audio_host)
        self.audio_thread.start()

    # 数据推送
    def push_data(self):
        self.command('chassis push position on attitude on freq 30')
        print('push data turn on!')
        self.data_thread.start()

    # 控制命令返回
    def ctrl_recv(self):
        while not rospy.is_shutdown():
            buff = self.ctrl_socket.recv(1024)
            msg = buff.decode('utf-8')
            # print('recv <- ', msg)

    # S1返回数据接收
    def data_recv(self):
        while not rospy.is_shutdown():
            buff, dest_info = self.data_socket.recvfrom(1024)
            buff = buff.decode('utf-8')
            msg = buff.split(' ')
            if buff.find('chassis') != -1:
                if buff.find('position') != -1 and buff.find('attitude') != -1:
                    x = float(msg[msg.index('position') + 1])
                    y = float(msg[msg.index('position') + 2])
                    pitch = float(msg[msg.index('attitude') + 1])/57.3
                    roll = float(msg[msg.index('attitude') + 2])/57.3
                    yaw = float(msg[msg.index('attitude') + 3])/57.3
                    self.br.sendTransform((x, -y, 0),
                                          tf.transformations.quaternion_from_euler(0, -0, -yaw),
                                          rospy.Time.now(),
                                          "base_link",
                                          "odom")

    # 事件接收
    def event_recv(self):
        while not rospy.is_shutdown():
            buff = self.event_socket.recv(1024)
            msg = buff.decode('utf-8')
            # print('recv <- ', msg)

    # 视频流接收
    def stream_recv(self):
        # s = open('./data', 'ab+')
        package_data = b''
        while not rospy.is_shutdown():
            buff = self.stream_socket.recv(2048)
            # s.write(buff)
            package_data += buff
            frames = self._h264_decode(package_data)
            for frame in frames:
                self.global_frame = frame
                self.pub_image()
            package_data = b''
        # s.close()

    # 音频流接收
    def audio_recv(self):

        s = open('./data', 'ab+')
        # package_data = ""
        while not rospy.is_shutdown():
            buff = self.audio_socket.recv(2048)
            s.write(buff)
            # package_data += buff
            # if len(buff) != 1460:
            #     package_data = ""
        s.close()

    # 视频流解码
    def _h264_decode(self, packet_data):
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                # print('frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls))
                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, w, 3)))
                res_frame_list.append(frame)

        return res_frame_list

    # SDK命令发送
    def command(self, msg):
        msg = msg + ';'
        # print('send -> ', msg)
        self.ctrl_socket.send(msg.encode('utf-8'))

    # 麦轮解算，提高控制精度为0.07m/s
    def move_with_wheel_speed(self, x=0.0, y=0.0, yaw=0.0):
        yaw = -yaw / 57.3
        a = 0.10
        b = 0.10
        r = 0.05
        pi = math.pi
        w0 = (x - y + yaw * (a + b)) * (30/pi) / r
        w1 = (x + y - yaw * (a + b)) * (30/pi) / r
        w2 = (x - y - yaw * (a + b)) * (30/pi) / r
        w3 = (x + y + yaw * (a + b)) * (30/pi) / r
        # print("wheel speed: ", w0, w1, w2, w3)
        self.command('chassis wheel w1 %f w2 %f w3 %f w4 %f' % (w0, w1, w2, w3))

    # 底盘速度控制
    def move_with_speed(self, x=0.0, y=0.0, yaw=0.0):
        self.command('chassis speed x %f y %f z %f' % (x, y, yaw))

    # 底盘位置控制
    def move_with_position(self, x=0.0, y=0.0, yaw=0.0):
        # print('chassis speed x %f y %f z %f'%(x, y, yaw))
        self.command('chassis move x %f y %f z %f wait_for_complete true' % (x, y, yaw))

    # 机械臂初始化
    def arm_init(self):
        self.command('robotic_arm recenter')

    # 机械臂以及机械爪控制
    def arm_grip_ctrl(self, x=0.0, y=0.0, flag=0):
        self.command('robotic_arm moveto x %f y %f' % (x, y))
        time.sleep(1)
        if(flag == 1):
            self.command('robotic_gripper open 4')
        if(flag == 0):
            self.command('robotic_gripper close 3')
            time.sleep(1)

    # 手柄数据回调
    def joy_callback(self, msg):
        self.axes = msg.axes
        # print(self.axes)

    # 速度回调
    def cmd_vel_callback(self, msg):
        self.cmd_vel[0] = msg.linear.x
        self.cmd_vel[1] = msg.linear.y
        self.cmd_vel[2] = msg.angular.z
        # print("cmd_vel:", self.cmd_vel)
        # self.move_with_speed(self.cmd_vel[0], -self.cmd_vel[1], -(self.cmd_vel[2] * 57.29578))
        self.move_with_wheel_speed(self.cmd_vel[0], -self.cmd_vel[1], -(self.cmd_vel[2] * 57.29578))

    # 物块对位的位置回调
    def item_cali_pos_callback(self, msg):
        if self.scanStart is True:
            # 判断id
            self.item_position[0] = msg.pose.position.x
            self.item_position[1] = -msg.pose.position.y
            self.item_position[2] = msg.pose.position.z
            (item_r, item_p, item_y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                                                 msg.pose.orientation.y,
                                                                                 msg.pose.orientation.z,
                                                                                 msg.pose.orientation.w])
            self.euler_rpy[0] = item_r * 57.29578
            self.euler_rpy[1] = item_p * 57.29578
            self.euler_rpy[2] = -(item_y * 57.29578 + 90)
            if abs(self.euler_rpy[2]) <= 3:
                self.scanSuccess = True
            # print(pos_x: ", self.item_position[0], "pos_y: ", self.item_position[1], "pos_yaw: ", self.euler_rpy[2])

    # 物块对位的速度回调
    def item_calib_vel_callback(self, msg):
        if self.scanSuccess is True:
            self.calib_vel[0] = msg.linear.x
            self.calib_vel[1] = -msg.linear.y
            self.calib_vel[2] = -msg.angular.z * 57.3
            # self.move_with_speed(self.calib_vel[0], self.calib_vel[1], self.calib_vel[2] )
            if msg.linear.z == 1:
                self.calibSuccess = True
            else:
                self.calibSuccess = False
        # print("calib_vel:", self.calib_vel)

    # 状态机处理流程
    def fsm_process(self):
        if self.statusNum == 0:
            self.fsm_path_plan()
        elif self.statusNum == 1:
            self.fsm_item_calib()
        elif self.statusNum == 2:
            self.fsm_item_grip()
        elif self.statusNum == 3:
            self.fsm_item_trans()
        elif self.statusNum == 4:
            self.fsm_item_release()

    # 导航
    def fsm_path_plan(self):
        if self.planSuccess is True:
            print("Plan success --- ready to scan")
            self.planSuccess = False
            self.scanSuccess = False
            self.statusNum = 1

    # 物块对准
    def fsm_item_calib(self):
        while self.scanSuccess is False:
            self.scanStart = True
            if self.axes[2] == -1:
                self.manual_control()
            else:
                self.scan_item()
        self.command('chassis wheel w1 0 w2 0 w3 0 w4 0')  # 扫描成功后，底盘置0
        self.scanStart = False
        print("Scan success --- ready to calib")
        while self.calibSuccess is False:
            if abs(self.calib_vel[0] - 0) < 1.0e-8 and abs(self.calib_vel[1] - 0) < 1.0e-8 and abs(self.calib_vel[2] - 0) < 1.0e-8:
                self.command('chassis wheel w1 0 w2 0 w3 0 w4 0')
            else:
                if self.axes[2] == -1:
                    self.manual_control()
                else:
                    self.move_with_speed(self.calib_vel[0], self.calib_vel[1], self.calib_vel[2])  # TODO:替换为麦轮解算

        print("Calib success --- ready to grip")
        self.calibSuccess = True
        self.statusNum = 2

    # 扫描四周，寻找物块
    def scan_item(self):
        self.move_with_speed(0, 0, 20)

    # 物块抓取
    def fsm_item_grip(self):
        self.arm_grip_ctrl(210, -60, 0)
        self.arm_grip_ctrl(140, 60, 0)

        print("Grip success --- ready to transport")
        self.statusNum = 3

    # 物块运输
    def fsm_item_trans(self):
        if self.planSuccess is True:
            print("Transport success --- ready to release")
            self.planSuccess = False
            self.statusNum = 4

    # 物块释放
    def fsm_item_release(self):
        self.arm_grip_ctrl(140, 60, 1)
        print("Release success --- ready to plan next")
        self.statusNum = 0

    # 发布图片
    def pub_image(self):
        if self.global_frame is None:
            return
        raw_img = PImage.fromarray(self.global_frame)
        cv_img = cv2.cvtColor(np.array(raw_img), cv2.COLOR_RGB2BGR)
        if self.image_pub.get_num_connections() != 0:
            try:
                caminfomsg = self.make_camera_msg()
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
                self.camInfo_pub.publish(caminfomsg)
            except CvBridgeError as e:
                print(e)

    # 关闭socket端口
    def exit(self):
        print('You choose to stop me.')
        self.command('chassis push position off attitude off')  # 关闭数据推送，以便下次正常开启
        self.data_socket.close()
        self.event_socket.close()
        self.stream_socket.close()
        self.event_socket.close()
        self.ctrl_socket.close()

        self.ctrl_thread.join()
        self.data_thread.join()
        self.event_thread.join()
        self.stream_thread.join()

    # 手动操作
    def manual_control(self):
        if (abs(self.axes[4] - 0) < 1.0e-8) and (abs(self.axes[3] - 0) < 1.0e-8) and (abs(self.axes[0] - 0)) < 1.0e-8:
            self.command('chassis wheel w1 0 w2 0 w3 0 w4 0')
        else:
            self.move_with_speed(self.axes[4], -self.axes[3], -self.axes[0] * 50)  # wireless xbox


if __name__ == '__main__':
    rospy.init_node('s1_test', anonymous=True)
    s1 = S1_ROS(wifi)
    s1.start()
    s1.video_on()
    # s1.audio_on()#TODO:Not finished yet
    s1.push_data()
    s1.arm_grip_ctrl(100, 90, 1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        s1.fsm_process()
        rate.sleep()
    s1.exit()
