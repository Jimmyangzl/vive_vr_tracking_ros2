from ament_index_python.packages import get_package_share_directory
import os
import yaml
import argparse
import time
import numpy as np


def get_parser():
    config_path = os.path.join(
        get_package_share_directory('vr_tracking'),
        'config',
        'config.yaml'
    )
    with open(config_path, 'r') as file:
        configs = yaml.safe_load(file)
    arg_names = list(configs.keys())
    parser = argparse.ArgumentParser(
                    prog='tracker_pub',
                    description='Publishes tracker pose w.r.t. initial pose')
    for arg_name in arg_names:
        parser.add_argument("--"+arg_name, help="(Option) Specify "+arg_name)
    # args = parser.parse_args()
    return parser

def print_tracker_data(tracker, interval):
    # Continuously print tracker pose data at the specified interval
    while True:
        start_time = time.time()
        # Get pose data for the tracker device and format as a string
        #pose_data = " ".join(["%.4f" % val for val in tracker.get_pose_euler()])
        pose_data = " ".join(["%.4f" % val for val in tracker.get_pose_quaternion()])
        # Print pose data in the same line
        print("\r" + pose_data, end="")
        # Calculate sleep time to maintain the desired interval
        sleep_time = interval - (time.time() - start_time)
        # Sleep if necessary
        if sleep_time > 0:
            time.sleep(sleep_time)

def rotation_matrix_to_quaternion(R):
    """
    Converts a rotation matrix to a quaternion.
    
    Parameters:
    R (numpy.ndarray): A 3x3 rotation matrix.
    
    Returns:
    numpy.ndarray: A quaternion [w, x, y, z].
    """
    # Ensure the matrix is 3x3
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"
    
    # Calculate the trace of the matrix
    trace = np.trace(R)
    
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * w
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4 * x
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4 * y
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4 * z
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    return np.array([w, x, y, z])