#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import tf.transformations
from rs import RS485Communicator, ModbusDevice

class MRobotDriver:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("mrobot_driver", anonymous=False)

        # 参数设置
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.095)         # 后轮半径（单位 m）
        self.wheel_separation = rospy.get_param("~wheel_separation", 0.515)   # 两后轮圆心间距（单位 m）
        self.conversion_ratio = rospy.get_param("~conversion_ratio", 6600)    # 换向频率（0.1Hz）与轮子转速（rps）的换算系数
        self.control_rate = rospy.get_param("~rate", 20)                      # 控制周期（Hz）

        # 初始化 cmd_vel 的期望速度
        self.cmd_vel_linear = 0.0   # 单位：m/s
        self.cmd_vel_angular = 0.0  # 单位：rad/s

        # 里程计初始状态（积分）
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_time = rospy.Time.now()

        # ROS 话题的发布和订阅
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # TF 广播器
        self.odom_broadcaster = tf.TransformBroadcaster()

        # 初始化 RS485 通讯及 Modbus 设备（串口参数可通过参数配置）
        serial_port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        baudrate = rospy.get_param("~baudrate", 9600)
        timeout = rospy.get_param("~timeout", 0.2)
        self.communicator = RS485Communicator(port=serial_port, baudrate=baudrate, timeout=timeout)
        # 左侧电机（从站地址 0x01）与右侧电机（从站地址 0x02）
        self.left_motor = ModbusDevice(self.communicator, 0x01)
        self.right_motor = ModbusDevice(self.communicator, 0x02)

        # 互斥锁保护共享数据（如 cmd_vel）
        self.lock = threading.Lock()

        # 设置里程计协方差（可根据实际情况调整）
        self.odom_pose_covariance = [
            1e-9, 0,    0,    0,    0,    0,
            0,    1e-3, 1e-9, 0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    1e-9
        ]
        self.odom_twist_covariance = [
            1e-9, 0,    0,    0,    0,    0,
            0,    1e-3, 1e-9, 0,    0,    0,
            0,    0,    1e6,  0,    0,    0,
            0,    0,    0,    1e6,  0,    0,
            0,    0,    0,    0,    1e6,  0,
            0,    0,    0,    0,    0,    1e-9
        ]

    def cmd_vel_callback(self, msg):
        with self.lock:
            self.cmd_vel_linear = msg.linear.x
            self.cmd_vel_angular = msg.angular.z

    def compute_wheel_speeds(self, v, omega):
        """
        根据差速运动学计算左右轮的线速度，然后转换为轮子转速（rps）
        v：期望线速度（m/s）
        omega：期望角速度（rad/s）
        返回：(left_rps, right_rps)
        """
        # 左轮和右轮的线速度（m/s）
        v_left = v - (omega * self.wheel_separation / 2.0)
        v_right = v + (omega * self.wheel_separation / 2.0)
        # 转换为轮子的转速（rps）
        left_rps = v_left / self.wheel_radius
        right_rps = v_right / self.wheel_radius
        return left_rps, right_rps

    def update_odometry(self, dt, left_rps_meas, right_rps_meas):
        """
        根据测量得到的左右轮转速（rps），计算机器人运动学信息并积分更新里程计
        """
        # 将轮子转速（rps）转换为线速度（m/s）
        v_left = left_rps_meas * self.wheel_radius
        v_right = right_rps_meas * self.wheel_radius
        # 机器人线速度和角速度（差速模型）
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_separation

        # 简单积分更新位姿
        delta_x = v * math.cos(self.odom_yaw) * dt
        delta_y = v * math.sin(self.odom_yaw) * dt
        delta_yaw = omega * dt

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_yaw += delta_yaw

        return v, omega

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt == 0:
                dt = 1.0 / self.control_rate

            # 获取最新的 cmd_vel 指令
            with self.lock:
                v_cmd = self.cmd_vel_linear
                omega_cmd = self.cmd_vel_angular

            # 计算左右轮目标转速（rps）
            left_rps_des, right_rps_des = self.compute_wheel_speeds(v_cmd, omega_cmd)
            # 根据转换系数，将 rps 转换为下位机指令值（单位：0.1Hz）
            left_motor_cmd = left_rps_des * self.conversion_ratio
            right_motor_cmd = right_rps_des * self.conversion_ratio

            # 将目标速度发送给下位机
            try:
                self.left_motor.write_speed(left_motor_cmd)
                self.right_motor.write_speed(right_motor_cmd)
            except Exception as e:
                rospy.logwarn("写入电机速度出错: %s, 写入速度： %d", e, round(left_motor_cmd))

            # 读取当前电机速度（这里使用换向频率版本，单位为0.1Hz，转换为 rps）
            try:
                left_speed_hz = self.left_motor.read_speed(hz=True)  # 左轮返回的换向频率（已处理符号）
                right_speed_hz = self.right_motor.read_speed(hz=True)
                left_rps_meas = left_speed_hz / self.conversion_ratio
                right_rps_meas = right_speed_hz / self.conversion_ratio
            except Exception as e:
                rospy.logwarn("读取电机速度出错: %s", e)
                left_rps_meas = 0.0
                right_rps_meas = 0.0

            # 更新里程计（使用测量得到的轮速进行积分）
            v_meas, omega_meas = self.update_odometry(dt, left_rps_meas, right_rps_meas)

            # 构造并发布 odom 消息
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.odom_yaw)
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]
            # 添加协方差矩阵，表达位置和速度的不确定性
            odom.pose.covariance = self.odom_pose_covariance
            odom.twist.covariance = self.odom_twist_covariance
            odom.twist.twist.linear.x = v_meas
            odom.twist.twist.angular.z = omega_meas
            self.odom_pub.publish(odom)

            # 通过 tf 广播器发布坐标变换：odom -> base_link
            self.odom_broadcaster.sendTransform(
                (self.odom_x, self.odom_y, 0),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            self.last_time = current_time
            rate.sleep()

    def close(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.communicator.close()

def main():
    driver = MRobotDriver()
    try:
        driver.run()
    except rospy.ROSInterruptException:
        raise
    finally:
        driver.close()
        print('运行结束')

if __name__ == '__main__':
    main()
