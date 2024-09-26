import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
import yaml
import math
import time
from ament_index_python.packages import get_package_share_directory
from vr_tracking.track import ViveTrackerModule
# from vive_tracker.render_argparse import *
import numpy as np
from vr_tracking.tracker_func import get_parser, print_tracker_data
from vr_tracking.tracker_func import rotation_matrix_to_quaternion

class TrackerPublisher(Node):
    def __init__(self, args_parser):
        super().__init__('tracker_pub')
        self.parser_args = args_parser.parse_args()
        self.vive_tracker = ViveTrackerModule()
        self.vive_tracker.print_discovered_objects()
        self.pose_pubs = {
            "left": None,
            "right": None,
        }
    
    def set_configuration(self):
        self.set_args()
        self.get_trackers()
        # R_t is used to rotate adjust tracker frame
        # right multiply to rotate tracker frame into: z points to palm, x points to fingers
        self.R_t = np.array([
            [0.0, 1.0, 0.0], 
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        return True

    def calibrate_frame(self):
        frame_cali_wait_time = 2
        self.get_logger().info(f"Frame calibration starts in {frame_cali_wait_time}s, please get ready.")
        time.sleep(frame_cali_wait_time)
        for tracker_name, tracker in self.trackers.items():
            if tracker:
                tracker.frame_calibrate()
                assert tracker.calibrate_flag, tracker_name + " frame calibration failed."
                self.get_logger().info(tracker_name + ' frame calibrated.')
                self.pose_pubs[tracker_name] = self.create_publisher(Float64MultiArray,
                                        getattr(self, tracker_name+"_topic"), 1)
        return True
    
    def start_timer(self):
        timer_period = 1.0 / float(self.freq)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer.cancel()
        for key in self.trackers:
            if self.trackers[key].calibrate_flag:
                self.timer.reset()
        return True
    
    def set_args(self):
        parser_args_dict = vars(self.parser_args)
        config_path = os.path.join(
            get_package_share_directory('vr_tracking'),
            'config',
            'config.yaml'
        )
        with open(config_path, 'r') as file:
            configs = yaml.safe_load(file)
        for key, value in configs.items():
            if parser_args_dict[key]:
                value = parser_args_dict[key]
            setattr(self, key, value)
            self.get_logger().info(key + ': ' + str(value))
        return True

    def get_trackers(self):
        self.trackers = {}
        for index, value in self.vive_tracker.devices.items():
            if "tracker" in index and value.get_serial() == self.left_tracker_serial:
                self.trackers['left'] = VRTracker(value)
            if "tracker" in index and value.get_serial() == self.right_tracker_serial:
                self.trackers['right'] = VRTracker(value)
        assert self.trackers, "No tracker connected."
        return True
    
    def timer_callback(self):
        for tracker_name, tracker in self.trackers.items():
            pose_b = np.array(list(tracker.tracker.get_pose_matrix()))
            pose_b[:3, :3] = pose_b[:3, :3] @ self.R_t
            pose_o = tracker.o_R_b @ pose_b
            pose_o[:3, -1] = (pose_o[:3, -1] - tracker.origin)
            for i in range(3):
                pose_o[i, -1] = tracker.position_filter[i].filter(pose_o[i, -1]*self.translational_gain)
            pose_msg = Float64MultiArray()
            if self.quat:
                orientation = rotation_matrix_to_quaternion(pose_o[:3, :3])
            else:
                orientation = pose_o[:3, :3].reshape(-1, order='F')
            pose = np.append(pose_o[:3, -1], orientation)
            pose_msg.data = pose.astype(np.float64).tolist()
            self.pose_pubs[tracker_name].publish(pose_msg)
        return True


class VRTracker:
    def __init__(self, tracker):
        self.tracker = tracker
        self.o_R_b = None
        self.origin = None
        self.calibrate_flag = False
        self.position_filter = [OneEuroFilter(
            min_cutoff=1.0, beta=0.01, d_cutoff=1.0, freq=50.0) for _ in range(3)]
        
    def frame_calibrate(self):
        try:
            b_pose_t = np.array(list(self.tracker.get_pose_matrix()))
            b_R_t = b_pose_t[:3, :3]
            # Get o_R_b, o: new original frame, b: base frame
            self.o_R_b = np.array([b_R_t[:, 0], -b_R_t[:, 1], -b_R_t[:, 2]]) 
            self.origin = self.o_R_b @ b_pose_t[:3, -1]
            self.calibrate_flag = True
        except:
            self.o_R_b = None
            self.origin = None
            self.calibrate_flag = False


class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0, freq=50.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.freq = freq
        self.x_prev = None
        self.dx_prev = None
        self.alpha = self.compute_alpha(min_cutoff)

    def compute_alpha(self, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * self.freq)

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = 0.0
            return x
        dx = (x - self.x_prev) * self.freq
        self.dx_prev = self.dx_prev + self.compute_alpha(self.d_cutoff) * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * abs(self.dx_prev)
        self.alpha = self.compute_alpha(cutoff)
        x_hat = self.x_prev + self.alpha * (x - self.x_prev)
        self.x_prev = x_hat
        return x_hat       


def main(args=None):
    args_parser = get_parser()
    rclpy.init(args=args)
    tracker_pub_node = TrackerPublisher(args_parser)
    tracker_pub_node.set_configuration()
    tracker_pub_node.calibrate_frame()
    tracker_pub_node.start_timer()
    rclpy.spin(tracker_pub_node)
    tracker_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
