import pytest
import argparse
import numpy as np
from vr_tracking.tracker_func import get_parser, rotation_matrix_to_quaternion


def test_get_parser():
    parser = argparse.ArgumentParser()
    parser_test = get_parser()
    assert type(parser_test)==type(parser), 'No parser.'

'''
def test_print_tracker_data():
    # This function is for debug only, to see the raw data from tracker
    pass
'''

def test_rotation_matrix_to_quaternion():
    rotation_matrix_identity = np.identity(3)
    quat = np.array([1.0, 0.0, 0.0, 0.0])
    assert (rotation_matrix_to_quaternion(
        rotation_matrix_identity)==quat).any(), 'Wrong quaternion conversion.'