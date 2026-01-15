#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, BoolStamped
from sensor_msgs.msg import Range

# Simple reusable PID controller class
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_int = 0.0
        self.prev_error = None

    def calc_p(self, error):
        return self.kp * error

    def calc_i(self, error, dt, res_err_int=False):
        if res_err_int:
            self.error_int = 0.0
        self.error_int += error * dt
        return self.ki * self.error_int

    def calc_d(self, error, dt):
        if self.prev_error is None or dt <= 0.0:
            self.prev_error = error
            return 0.0
        d_err = error - self.prev_error
        self.prev_error = error
        return self.kd * d_err / dt


class Controller:
    def __init__(self):
        rospy.set_param("controller_ready", "true")

        # --- Subscriptions ---
        # Lane pose (from lane_filter_node)
        rospy.Subscriber("lane_filter_node/lane_pose",
                         LanePose,
                         callback=self.lane_callback)

        # Vehicle detection flag (from VehicleDetectionNode)
        # Topic: /<veh>/vehicle_detection_node/detection
        rospy.Subscriber("vehicle_detection_node/detection",
                         BoolStamped,
                         callback=self.detection_callback)

        # Front ToF range (in meters)
        # Topic: /<veh>/front_center_tof_driver_node/range
        rospy.Subscriber("front_center_tof_driver_node/range",
                         Range,
                         callback=self.range_callback)

        # --- Publisher: velocity commands ---
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd",
                                   Twist2DStamped,
                                   queue_size=10)

        # --- PID setup (lane keeping) ---
        self.kp = rospy.get_param("kp", 3.0)
        self.ki = rospy.get_param("ki", 0.0)
        self.kd = rospy.get_param("kd", 0.5)

        self.pid_controller = PID(self.kp, self.ki, self.kd)
        self.prev_time = None

        # --- Obstacle detection logic ---
        self.vehicle_detected = False        # from vehicle_detection_node
        self.obstacle_distance = None        # from ToF

        # Trigger and clear thresholds for distance (meters)
        self.obstacle_threshold = rospy.get_param("obstacle_threshold", 0.3)         # trigger
        self.obstacle_threshold_clear = rospy.get_param("obstacle_threshold_clear", 0.3)  # clear

        # Flag to avoid re-triggering on same obstacle
        self.obstacle_blocked = False

        # --- State machine for behavior ---
        # LANE_FOLLOW -> STOP -> TURN_LEFT -> TURN_RIGHT -> LANE_FOLLOW
        self.state = "LANE_FOLLOW"
        self.state_start_time = rospy.Time.now()

        # --- Avoidance maneuver parameters ---
        self.stop_duration = rospy.get_param("stop_duration", 1.0)         # seconds
        self.turn_left_duration = rospy.get_param("turn_left_duration", 1.5)
        self.turn_right_duration = rospy.get_param("turn_right_duration", 1.0)

        self.turn_v = rospy.get_param("turn_v", 0.15)                      # linear vel during turns
        self.turn_left_omega = rospy.get_param("turn_left_omega", 2.0)     # rad/s (+ = left)
        self.turn_right_omega = rospy.get_param("turn_right_omega", -2.0)  # rad/s (- = right)

        rospy.loginfo("Avoidance Controller initialized (vehicle detection + ToF distance)")

    # ----------------- Callbacks -----------------

    def detection_callback(self, msg: BoolStamped):
        """From vehicle_detection_node/detection: True if a Duckiebot pattern is detected."""
        self.vehicle_detected = bool(msg.data)

    def range_callback(self, msg: Range):
        """Front ToF distance in meters."""
        r = msg.range
        self.obstacle_distance = r

        # Optional: ignore obviously invalid readings
        if r <= 0.0 or r > 3.0:
            return

        # If obstacle is clearly far away: reset the blocked flag
        if r > self.obstacle_threshold_clear:
            if self.obstacle_blocked:
                rospy.loginfo(f"Obstacle cleared at {r:.2f} m")
            self.obstacle_blocked = False
            return

        # If we are already handling an obstacle, do NOT retrigger
        if self.obstacle_blocked:
            return

        # NEW trigger condition:
        #   - A Duckiebot is detected in front
        #   - AND its distance is below the threshold
        #   - AND we are currently lane following
        if (self.vehicle_detected and
                r <= self.obstacle_threshold and
                self.state == "LANE_FOLLOW"):

            rospy.loginfo(f"NEW Duckiebot obstacle at {r:.2f} m: switching to STOP state")
            self.obstacle_blocked = True
            self.state = "STOP"
            self.state_start_time = rospy.Time.now()

            # Reset PID so there is no big jump afterwards
            self.pid_controller.error_int = 0.0
            self.pid_controller.prev_error = None

    def lane_callback(self, msg: LanePose):
        """Main control callback: uses state machine to decide commands."""
        if self.prev_time is None:
            self.prev_time = rospy.Time.now()

        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        if dt <= 0.0:
            dt = 1e-3
        self.prev_time = now

        control_msg = Twist2DStamped()

        # ---------------- STATE MACHINE ----------------
        if self.state == "STOP":
            # Hard stop
            control_msg.v = 0.0
            control_msg.omega = 0.0

            if (now - self.state_start_time).to_sec() > self.stop_duration:
                rospy.loginfo("STOP complete, switching to TURN_LEFT")
                self.state = "TURN_LEFT"
                self.state_start_time = now

        elif self.state == "TURN_LEFT":
            # Go around the obstacle: left arc
            control_msg.v = self.turn_v
            control_msg.omega = self.turn_left_omega

            elapsed = (now - self.state_start_time).to_sec()
            if elapsed > self.turn_left_duration:
                rospy.loginfo("TURN_LEFT complete, switching to TURN_RIGHT (reorient)")
                self.state = "TURN_RIGHT"
                self.state_start_time = now

        elif self.state == "TURN_RIGHT":
            # Re-orient back towards the lane
            control_msg.v = self.turn_v
            control_msg.omega = self.turn_right_omega

            elapsed = (now - self.state_start_time).to_sec()
            # Exit TURN_RIGHT only when:
            #   - we have turned long enough AND
            #   - we are no longer blocked by the obstacle
            if elapsed > self.turn_right_duration and not self.obstacle_blocked:
                rospy.loginfo("TURN_RIGHT complete, returning to LANE_FOLLOW")
                self.state = "LANE_FOLLOW"
                self.state_start_time = now

                # Reset PID memory before resuming lane keeping
                self.pid_controller.error_int = 0.0
                self.pid_controller.prev_error = None

        else:  # self.state == "LANE_FOLLOW"
            # Normal lane-following PID
            control_msg.v = 0.2
            error = -1.0 * msg.phi  # same convention as before

            rospy.loginfo(f"[LANE_FOLLOW] phi error: {error:.4f}")

            p_term = self.pid_controller.calc_p(error)
            i_term = self.pid_controller.calc_i(error, dt)
            d_term = self.pid_controller.calc_d(error, dt)

            control_msg.omega = p_term + i_term + d_term
        # ---------------- END STATE MACHINE ----------------

        rospy.loginfo(
            f"State: {self.state}, cmd: v={control_msg.v:.3f}, omega={control_msg.omega:.3f}"
        )
        self.pub.publish(control_msg)

    def send_zero_commands(self):
        control_msg = Twist2DStamped()
        control_msg.v = 0.0
        control_msg.omega = 0.0
        self.pub.publish(control_msg)


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    rospy.loginfo("Controller Node Starting (lane keeping + Duckiebot avoidance)")
    ct = Controller()
    rospy.on_shutdown(ct.send_zero_commands)
    rospy.spin()#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, BoolStamped
from sensor_msgs.msg import Range

# Simple reusable PID controller class
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_int = 0.0
        self.prev_error = None

    def calc_p(self, error):
        return self.kp * error

    def calc_i(self, error, dt, res_err_int=False):
        if res_err_int:
            self.error_int = 0.0
        self.error_int += error * dt
        return self.ki * self.error_int

    def calc_d(self, error, dt):
        if self.prev_error is None or dt <= 0.0:
            self.prev_error = error
            return 0.0
        d_err = error - self.prev_error
        self.prev_error = error
        return self.kd * d_err / dt


class Controller:
    def __init__(self):
        rospy.set_param("controller_ready", "true")

        # --- Subscriptions ---
        # Lane pose (from lane_filter_node)
        rospy.Subscriber("lane_filter_node/lane_pose",
                         LanePose,
                         callback=self.lane_callback)

        # Vehicle detection flag (from VehicleDetectionNode)
        # Topic: /<veh>/vehicle_detection_node/detection
        rospy.Subscriber("vehicle_detection_node/detection",
                         BoolStamped,
                         callback=self.detection_callback)

        # Front ToF range (in meters)
        # Topic: /<veh>/front_center_tof_driver_node/range
        rospy.Subscriber("front_center_tof_driver_node/range",
                         Range,
                         callback=self.range_callback)

        # --- Publisher: velocity commands ---
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd",
                                   Twist2DStamped,
                                   queue_size=10)

        # --- PID setup (lane keeping) ---
        self.kp = rospy.get_param("kp", 5.0)
        self.ki = rospy.get_param("ki", 0.1)
        self.kd = rospy.get_param("kd", 1.0)

        self.pid_controller = PID(self.kp, self.ki, self.kd)
        self.prev_time = None

        # --- Obstacle detection logic ---
        self.vehicle_detected = False        # from vehicle_detection_node
        self.obstacle_distance = None        # from ToF

        # Trigger and clear thresholds for distance (meters)
        self.obstacle_threshold = rospy.get_param("obstacle_threshold", 0.2)         # trigger
        self.obstacle_threshold_clear = rospy.get_param("obstacle_threshold_clear", 0.3)  # clear

        # Flag to avoid re-triggering on same obstacle
        self.obstacle_blocked = False

        # --- State machine for behavior ---
        # LANE_FOLLOW -> STOP -> TURN_LEFT -> TURN_RIGHT -> LANE_FOLLOW
        self.state = "LANE_FOLLOW"
        self.state_start_time = rospy.Time.now()

        # --- Avoidance maneuver parameters ---
        self.stop_duration = rospy.get_param("stop_duration", 1.5)         # seconds
        self.turn_left_duration = rospy.get_param("turn_left_duration", 1.2)
        self.turn_right_duration = rospy.get_param("turn_right_duration", 1.2)

        self.turn_v = rospy.get_param("turn_v", 0.15)                      # linear vel during turns
        self.turn_left_omega = rospy.get_param("turn_left_omega", 1.5)     # rad/s (+ = left)
        self.turn_right_omega = rospy.get_param("turn_right_omega", -1.5)  # rad/s (- = right)

        rospy.loginfo("Avoidance Controller initialized (vehicle detection + ToF distance)")

    # ----------------- Callbacks -----------------

    def detection_callback(self, msg: BoolStamped):
        """From vehicle_detection_node/detection: True if a Duckiebot pattern is detected."""
        self.vehicle_detected = bool(msg.data)

    def range_callback(self, msg: Range):
        """Front ToF distance in meters."""
        r = msg.range
        self.obstacle_distance = r

        # Optional: ignore obviously invalid readings
        if r <= 0.0 or r > 3.0:
            return

        # If obstacle is clearly far away: reset the blocked flag
        if r > self.obstacle_threshold_clear:
            if self.obstacle_blocked:
                rospy.loginfo(f"Obstacle cleared at {r:.2f} m")
            self.obstacle_blocked = False
            return

        # If we are already handling an obstacle, do NOT retrigger
        if self.obstacle_blocked:
            return

        # NEW trigger condition:
        #   - A Duckiebot is detected in front
        #   - AND its distance is below the threshold
        #   - AND we are currently lane following
        if (self.vehicle_detected and
                r <= self.obstacle_threshold and
                self.state == "LANE_FOLLOW"):

            rospy.loginfo(f"NEW Duckiebot obstacle at {r:.2f} m: switching to STOP state")
            self.obstacle_blocked = True
            self.state = "STOP"
            self.state_start_time = rospy.Time.now()

            # Reset PID so there is no big jump afterwards
            self.pid_controller.error_int = 0.0
            self.pid_controller.prev_error = None

    def lane_callback(self, msg: LanePose):
        """Main control callback: uses state machine to decide commands."""
        if self.prev_time is None:
            self.prev_time = rospy.Time.now()

        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        if dt <= 0.0:
            dt = 1e-3
        self.prev_time = now

        control_msg = Twist2DStamped()

        # ---------------- STATE MACHINE ----------------
        if self.state == "STOP":
            # Hard stop
            control_msg.v = 0.0
            control_msg.omega = 0.0

            if (now - self.state_start_time).to_sec() > self.stop_duration:
                rospy.loginfo("STOP complete, switching to TURN_LEFT")
                self.state = "TURN_LEFT"
                self.state_start_time = now

        elif self.state == "TURN_LEFT":
            # Go around the obstacle: left arc
            control_msg.v = self.turn_v
            control_msg.omega = self.turn_left_omega

            elapsed = (now - self.state_start_time).to_sec()
            if elapsed > self.turn_left_duration:
                rospy.loginfo("TURN_LEFT complete, switching to TURN_RIGHT (reorient)")
                self.state = "TURN_RIGHT"
                self.state_start_time = now

        elif self.state == "TURN_RIGHT":
            # Re-orient back towards the lane
            control_msg.v = self.turn_v
            control_msg.omega = self.turn_right_omega

            elapsed = (now - self.state_start_time).to_sec()
            # Exit TURN_RIGHT only when:
            #   - we have turned long enough AND
            #   - we are no longer blocked by the obstacle
            if elapsed > self.turn_right_duration and not self.obstacle_blocked:
                rospy.loginfo("TURN_RIGHT complete, returning to LANE_FOLLOW")
                self.state = "LANE_FOLLOW"
                self.state_start_time = now

                # Reset PID memory before resuming lane keeping
                self.pid_controller.error_int = 0.0
                self.pid_controller.prev_error = None

        else:  # self.state == "LANE_FOLLOW"
            # Normal lane-following PID
            control_msg.v = 0.2
            error = -1.0 * msg.phi  # same convention as before

            rospy.loginfo(f"[LANE_FOLLOW] phi error: {error:.4f}")

            p_term = self.pid_controller.calc_p(error)
            i_term = self.pid_controller.calc_i(error, dt)
            d_term = self.pid_controller.calc_d(error, dt)

            control_msg.omega = p_term + i_term + d_term
        # ---------------- END STATE MACHINE ----------------

        rospy.loginfo(
            f"State: {self.state}, cmd: v={control_msg.v:.3f}, omega={control_msg.omega:.3f}"
        )
        self.pub.publish(control_msg)

    def send_zero_commands(self):
        control_msg = Twist2DStamped()
        control_msg.v = 0.0
        control_msg.omega = 0.0
        self.pub.publish(control_msg)


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    rospy.loginfo("Controller Node Starting (lane keeping + Duckiebot avoidance)")
    ct = Controller()
    rospy.on_shutdown(ct.send_zero_commands)
    rospy.spin()