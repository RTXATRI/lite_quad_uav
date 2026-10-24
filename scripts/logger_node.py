#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from lite_quad_uav.msg import (
    ConnectManagerLogReport,
    ControllerLogReport,
    SafetySupervisorLogReport,
    TaskManagerLogReport,
    TaskNodeLogReport,
    TargetEstimatorLogReport,
    VelocityManagerLogReport,
)


class LogReportCache(object):
    def __init__(self):
        self.print_rate = rospy.get_param("~print_rate", 1.0)
        self.report_timeout = rospy.get_param("~report_timeout", 3.0)
        self.show_aux = self._as_bool(rospy.get_param("~show_aux", False))
        self.show_inactive_tasks = self._as_bool(rospy.get_param("~show_inactive_tasks", False))
        self.max_message_len = self._as_int(rospy.get_param("~max_message_len", 48), 48)
        self.compact = self._as_bool(rospy.get_param("~compact", False))
        self.show_target_estimator = self._as_bool(rospy.get_param("~show_target_estimator", False))
        self.task_ids = rospy.get_param("~task_ids", [])
        self.aux_ids = rospy.get_param("~aux_ids", [])

        if self.print_rate <= 0.0:
            rospy.logwarn("[Logger] print_rate must be positive, reset to 1.0")
            self.print_rate = 1.0
        if self.report_timeout <= 0.0:
            rospy.logwarn("[Logger] report_timeout must be positive, reset to 3.0")
            self.report_timeout = 3.0
        if self.max_message_len <= 0:
            rospy.logwarn("[Logger] max_message_len must be positive, reset to 48")
            self.max_message_len = 48

        self.reports = {}
        self.subscribers = []

        # ── 话题名称 ──────────────────────────────────────────────────
        self.TOPIC_CONNECT_MANAGER = '/uav/log_reports/connect_manager'   # connect_manager 日志报告
        self.TOPIC_VELOCITY_MANAGER = '/uav/log_reports/velocity_manager' # velocity_manager 日志报告
        self.TOPIC_CONTROLLER = '/uav/log_reports/controller'             # controller 日志报告
        self.TOPIC_SAFETY_SUPERVISOR = '/uav/log_reports/safety_supervisor' # safety_supervisor 日志报告
        self.TOPIC_TASK_MANAGER = '/uav/log_reports/task_manager'         # task_manager 日志报告
        self.TOPIC_TARGET_ESTIMATOR = '/uav/log_reports/target_estimator' # target_estimator 日志报告
        self.TOPIC_TASK_PREFIX = '/uav/log_reports/tasks/'               # 任务节点日志话题前缀
        self.TOPIC_AUX_PREFIX = '/uav/log_reports/aux/'                  # 辅助节点日志话题前缀

        self._subscribe_reports()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.print_rate), self._print_timer_cb)

    def _subscribe_reports(self):
        self._subscribe(
            "connect_manager",
            self.TOPIC_CONNECT_MANAGER,
            ConnectManagerLogReport,
            self._format_connect_manager,
        )
        self._subscribe(
            "velocity_manager",
            self.TOPIC_VELOCITY_MANAGER,
            VelocityManagerLogReport,
            self._format_velocity_manager,
        )
        self._subscribe(
            "controller",
            self.TOPIC_CONTROLLER,
            ControllerLogReport,
            self._format_controller,
        )
        self._subscribe(
            "safety_supervisor",
            self.TOPIC_SAFETY_SUPERVISOR,
            SafetySupervisorLogReport,
            self._format_safety_supervisor,
        )
        self._subscribe(
            "task_manager",
            self.TOPIC_TASK_MANAGER,
            TaskManagerLogReport,
            self._format_task_manager,
        )
        if self.show_target_estimator:
            self._subscribe(
                "target_estimator",
                self.TOPIC_TARGET_ESTIMATOR,
                TargetEstimatorLogReport,
                self._format_target_estimator,
            )

        for task_id in self.task_ids:
            self._subscribe(
                "task:%s" % task_id,
                self.TOPIC_TASK_PREFIX + task_id,
                TaskNodeLogReport,
                self._format_task_node,
            )

        for aux_id in self.aux_ids:
            self._subscribe(
                "aux:%s" % aux_id,
                self.TOPIC_AUX_PREFIX + aux_id,
                TaskNodeLogReport,
                self._format_task_node,
            )

    def _subscribe(self, key, topic, msg_type, formatter):
        self.reports[key] = {
            "topic": topic,
            "msg": None,
            "formatter": formatter,
        }
        self.subscribers.append(
            rospy.Subscriber(topic, msg_type, self._report_cb, callback_args=key, queue_size=10)
        )

    def _report_cb(self, msg, key):
        self.reports[key]["msg"] = msg

    def _age_sec(self, msg, now):
        if msg is None or msg.header.stamp == rospy.Time(0):
            return None
        return (now - msg.header.stamp).to_sec()

    def _entry_state(self, key, now):
        entry = self.reports.get(key)
        if entry is None:
            return "missing", None

        msg = entry["msg"]
        if msg is None:
            return "missing", None

        age = self._age_sec(msg, now)
        if age is None or age > self.report_timeout:
            return "stale", age

        return "ok", age

    def _state_text(self, state, age):
        if state == "missing":
            return "MISS"
        if state == "stale":
            return "STALE(unknown)" if age is None else "STALE(%.1fs)" % age
        return "OK"

    def _report_counts(self, now):
        counts = {"ok": 0, "stale": 0, "missing": 0}
        for key in self.reports.keys():
            state, _age = self._entry_state(key, now)
            counts[state] += 1
        return counts

    def _fresh_line(self, label, key, now, formatter):
        entry = self.reports.get(key)
        state, age = self._entry_state(key, now)

        if state != "ok":
            topic = entry["topic"] if entry is not None else "-"
            return "%-5s %s topic=%s" % (label, self._state_text(state, age), topic)

        return "%-5s %s" % (label, formatter(entry["msg"], age))

    def _report_summary(self, key, now, formatter):
        entry = self.reports.get(key)
        state, age = self._entry_state(key, now)

        if state != "ok":
            return self._state_text(state, age)

        return formatter(entry["msg"], age)

    def _print_timer_cb(self, _event):
        now = rospy.Time.now()
        counts = self._report_counts(now)
        header = "[UAV] t=%.1fs  reports: ok=%d stale=%d missing=%d" % (
            now.to_sec(),
            counts["ok"],
            counts["stale"],
            counts["missing"],
        )

        lines = [
            header,
            "",
            self._fresh_line("SYS", "connect_manager", now, self._format_connect_manager),
            self._fresh_line("SAFE", "safety_supervisor", now, self._format_safety_supervisor),
            self._fresh_line("CTRL", "controller", now, self._format_controller),
            self._fresh_line("VEL", "velocity_manager", now, self._format_velocity_manager),
            self._task_dashboard_line(now),
        ]
        if self.show_target_estimator:
            lines.insert(4, self._fresh_line("SENSE", "target_estimator", now, self._format_target_estimator))
        lines.extend(self._extra_task_lines(now))
        lines.extend(self._aux_lines(now))

        if self.compact:
            lines = [line for line in lines if line]
            rospy.loginfo(" | ".join(lines))
        else:
            rospy.loginfo("\n".join(lines))

    @staticmethod
    def yn(value):
        return "Y" if value else "N"

    @staticmethod
    def empty(value):
        return value if value else "-"

    def trim(self, text):
        text = self.empty(text)
        if len(text) <= self.max_message_len:
            return text
        if self.max_message_len <= 3:
            return text[:self.max_message_len]
        return text[: self.max_message_len - 3] + "..."

    @staticmethod
    def vel4(vx, vy, vz, yaw):
        return "(%.2f,%.2f,%.2f,%.2f)" % (vx, vy, vz, yaw)

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _as_int(value, default):
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _format_connect_manager(self, msg, _age):
        return (
            "state=%s ready=%s armed=%s mode=%s fcu=%s pos=%s vm=%s offb=%s"
            % (
                self.empty(msg.state),
                self.yn(msg.ready),
                self.yn(msg.armed),
                self.empty(msg.mode),
                self.yn(msg.fcu_connected),
                self.yn(msg.local_position_fresh),
                self.yn(msg.velocity_manager_alive),
                self.yn(msg.offboard_confirmed),
            )
        )

    def _format_velocity_manager(self, msg, _age):
        return (
            "state=%s src=%s watchdog=%s cand=%s out=%s"
            % (
                self.empty(msg.state),
                self.empty(msg.command_source),
                self.yn(msg.watchdog_active),
                self.yn(msg.candidate_fresh),
                self.vel4(msg.output_vx, msg.output_vy, msg.output_vz, msg.output_yaw_rate),
            )
        )

    def _format_controller(self, msg, _age):
        return (
            "node=%s state=%s rel=%s vstat=%s pos=%s src=%s/%s err=%.2f conv=%s sat=%s fallback=%s out=%s"
            % (
                self.empty(msg.node_state),
                self.empty(msg.state),
                self.yn(msg.reliable_vehicle_connection),
                self.yn(msg.vehicle_status_fresh),
                self.yn(msg.local_position_fresh),
                self.empty(msg.source_task),
                self.empty(msg.source_action),
                msg.position_error_norm,
                self.yn(msg.converged),
                self.yn(msg.saturated),
                self.yn(msg.fallback_active),
                self.vel4(msg.output_vx, msg.output_vy, msg.output_vz, msg.output_yaw_rate),
            )
        )

    def _format_safety_supervisor(self, msg, _age):
        return (
            "level=%s safe=%s battery=%s pos=%s watchdog=%s reason=%s"
            % (
                self.empty(msg.level),
                self.yn(msg.safe_to_fly),
                self.yn(msg.battery_fresh),
                self.yn(msg.local_position_fresh),
                self.yn(msg.velocity_watchdog_active),
                self.trim(msg.reason),
            )
        )

    def _format_task_manager(self, msg, _age):
        return (
            "mgr=%s accepting=%s busy=%s active=%s state=%s progress=%s action=%s registered=%d ready=%s"
            % (
                self.empty(msg.node_state),
                self.yn(msg.accepting_new_task),
                self.yn(msg.busy),
                self.empty(msg.active_task_id),
                self.empty(msg.active_task_state),
                self.empty(msg.active_task_progress),
                self.empty(msg.active_action),
                msg.registered_task_count,
                self.yn(msg.vehicle_ready),
            )
        )

    def _format_target_estimator(self, msg, _age):
        return (
            "node=%s rel=%s vstat=%s tag=%s fresh=%s det=%d tracked=%d valid=%d frame=%s msg=%s"
            % (
                self.empty(msg.node_state),
                self.yn(msg.reliable_vehicle_connection),
                self.yn(msg.vehicle_status_fresh),
                self.yn(msg.tag_detection_received),
                self.yn(msg.tag_detection_fresh),
                msg.detected_count,
                msg.tracked_count,
                msg.valid_count,
                self.empty(msg.body_frame),
                self.trim(msg.message),
            )
        )

    def _format_task_node(self, msg, _age):
        return (
            "%s node=%s rel=%s state=%s progress=%s action=%s msg=%s"
            % (
                self.empty(msg.task_name),
                self.empty(msg.node_state),
                self.yn(msg.reliable_vehicle_connection),
                self.empty(msg.task_state),
                self.empty(msg.task_progress),
                self.empty(msg.active_action),
                self.trim(msg.message),
            )
        )

    def _task_dashboard_line(self, now):
        task_manager = self._report_summary("task_manager", now, self._format_task_manager)
        if not self.task_ids:
            return "%-5s %s" % ("TASK", task_manager)

        task_id = self._selected_task_id(now)
        task_summary = self._report_summary("task:%s" % task_id, now, self._format_task_node)
        return "%-5s %s | %s" % ("TASK", task_manager, task_summary)

    def _selected_task_id(self, now):
        state, _age = self._entry_state("task_manager", now)
        if state == "ok":
            msg = self.reports["task_manager"]["msg"]
            if msg.active_task_id:
                return msg.active_task_id
        if self.task_ids:
            return self.task_ids[0]
        return ""

    def _extra_task_lines(self, now):
        lines = []
        selected_task_id = self._selected_task_id(now)
        for task_id in self.task_ids:
            if task_id == selected_task_id:
                continue

            key = "task:%s" % task_id
            state, _age = self._entry_state(key, now)
            msg = self.reports.get(key, {}).get("msg")
            should_show = self.show_inactive_tasks or state == "stale"
            should_show = should_show or (msg is not None and msg.task_state in ("RUNNING", "PAUSED", "FAILED", "CANCELED"))
            if not should_show:
                continue

            lines.append("%-5s %s" % ("TASK+", self._report_summary(key, now, self._format_task_node)))
        return lines

    def _aux_lines(self, now):
        if not self.show_aux:
            return []

        lines = []
        for aux_id in self.aux_ids:
            key = "aux:%s" % aux_id
            lines.append("%-5s %s" % ("AUX", self._report_summary(key, now, self._format_task_node)))
        return lines


def main():
    rospy.init_node("logger_node")
    cache = LogReportCache()
    rospy.loginfo("[Logger] logger_node running")
    rospy.spin()
    return cache


if __name__ == "__main__":
    main()
