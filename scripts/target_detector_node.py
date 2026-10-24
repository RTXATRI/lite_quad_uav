#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from lite_quad_uav.msg import TaskNodeLogReport

def publish_log_report(event):
    msg = TaskNodeLogReport()
    msg.header.stamp = rospy.Time.now()
    msg.task_name = 'target_detector'
    msg.node_state = 'NORMAL'
    msg.task_state = 'IDLE'
    msg.task_progress = 'PLACEHOLDER'
    msg.active_action = ''
    msg.local_position_fresh = False
    msg.vehicle_status_fresh = False
    msg.reliable_vehicle_connection = False
    msg.vehicle_ready = False
    msg.vehicle_armed = False
    msg.message = 'target_detector placeholder running'
    log_report_pub.publish(msg)

def main():
    global log_report_pub
    rospy.init_node('target_detector_node')
    log_report_rate = rospy.get_param('~log_report_rate', 1.0)
    if log_report_rate <= 0.0:
        rospy.logwarn('[TargetDetector] log_report_rate must be positive, reset to 1.0')
        log_report_rate = 1.0
    log_report_pub = rospy.Publisher('/uav/log_reports/aux/target_detector', TaskNodeLogReport, queue_size=10)  # 目标检测器节点日志报告
    rospy.Timer(rospy.Duration(1.0 / log_report_rate), publish_log_report)
    rospy.loginfo('uav_sensing placeholder running')
    rospy.spin()

if __name__ == '__main__':
    main()
