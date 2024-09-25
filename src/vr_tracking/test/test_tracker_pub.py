import pytest
import argparse
import numpy as np
import rclpy
from vr_tracking.tracker_func import get_parser
from vr_tracking.tracker_pub import TrackerPublisher


def test_tracker_publisher():
    if not rclpy.ok():
        rclpy.init()
    assert TrackerPublisher(get_parser()), 'Failed to initialize TrackerPublisher'
    rclpy.shutdown()

def test_set_args():
    if not rclpy.ok():
        rclpy.init()
    tracker_publisher_node = TrackerPublisher(get_parser())
    assert tracker_publisher_node.set_args(), 'set_args error.'
    tracker_publisher_node.destroy_node()
    rclpy.shutdown()

def test_get_trackers():
    if not rclpy.ok():
        rclpy.init()
    tracker_publisher_node = TrackerPublisher(get_parser())
    tracker_publisher_node.set_args()
    assert tracker_publisher_node.get_trackers(), 'get_trackers error.'
    tracker_publisher_node.destroy_node()
    rclpy.shutdown()

def test_calibrate_frame():
    if not rclpy.ok():
        rclpy.init()
    tracker_publisher_node = TrackerPublisher(get_parser())
    tracker_publisher_node.set_configuration()
    assert tracker_publisher_node.calibrate_frame(), 'calibrate_frame error.'
    tracker_publisher_node.destroy_node()
    rclpy.shutdown()

def test_start_timer():
    if not rclpy.ok():
        rclpy.init()
    tracker_publisher_node = TrackerPublisher(get_parser())
    tracker_publisher_node.set_configuration()
    tracker_publisher_node.calibrate_frame()
    assert tracker_publisher_node.start_timer(), 'start_timer error.'
    tracker_publisher_node.destroy_node()
    rclpy.shutdown()

def test_timer_callback():
    if not rclpy.ok():
        rclpy.init()
    tracker_publisher_node = TrackerPublisher(get_parser())
    tracker_publisher_node.set_configuration()
    tracker_publisher_node.calibrate_frame()
    tracker_publisher_node.start_timer()
    assert tracker_publisher_node.timer_callback(), 'timer_callback error.'
    tracker_publisher_node.destroy_node()
    rclpy.shutdown()

