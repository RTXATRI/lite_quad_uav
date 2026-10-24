#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from lite_quad_uav.msg import TaskNodeLogReport

# TODO: Placeholder classes for future GCS bridge implementation
class CommandBridge:
    """Bridge for receiving task commands from GCS"""
    pass

class ParamBridge:
    """Bridge for receiving algorithm parameters from GCS"""
    pass

class TelemetryBridge:
    """Bridge for sending telemetry data to GCS"""
    pass

class ProtocolAdapter:
    """Adapter for GCS communication protocol"""
    pass

def publish_log_report(event):
    msg = TaskNodeLogReport()
    msg.header.stamp = rospy.Time.now()
    msg.task_name = 'gcs_bridge'
    msg.node_state = 'NORMAL'
    msg.task_state = 'IDLE'
    msg.task_progress = 'PLACEHOLDER'
    msg.active_action = ''
    msg.local_position_fresh = False
    msg.vehicle_status_fresh = False
    msg.reliable_vehicle_connection = False
    msg.vehicle_ready = False
    msg.vehicle_armed = False
    msg.message = 'gcs_bridge placeholder running'
    log_report_pub.publish(msg)

def main():
    global log_report_pub
    rospy.init_node('gcs_bridge_node')
    log_report_rate = rospy.get_param('~log_report_rate', 1.0)
    if log_report_rate <= 0.0:
        rospy.logwarn('[GcsBridge] log_report_rate must be positive, reset to 1.0')
        log_report_rate = 1.0
    log_report_pub = rospy.Publisher('/uav/log_reports/aux/gcs_bridge', TaskNodeLogReport, queue_size=10)  # GCS 桥接节点日志报告
    rospy.Timer(rospy.Duration(1.0 / log_report_rate), publish_log_report)
    rospy.loginfo('gcs_bridge placeholder running')
    rospy.spin()

if __name__ == '__main__':
    main()
