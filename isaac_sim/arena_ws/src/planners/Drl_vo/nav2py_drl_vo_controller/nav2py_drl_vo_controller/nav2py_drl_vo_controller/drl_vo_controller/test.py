import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import unittest
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def find_closest_point(path, x, seg=-1):
    """
    Given a global path (of type Path) and a point x (np.array([x, y])),
    find the closest point on the path.
    If seg == -1, the function iterates over all segments; otherwise, it computes on a specific segment.
    Returns:
      - pt_min: the closest point (np.array([x, y]))
      - dist_min: the distance from x to pt_min
      - seg_min: the index of the segment where the closest point lies.
    """
    pt_min = np.array([np.nan, np.nan])
    dist_min = np.inf
    seg_min = -1

    if seg == -1:
        for i in range(len(path.poses) - 1):
            pt, dist, s = find_closest_point(path, x, i)
            if dist < dist_min:
                pt_min = pt
                dist_min = dist
                seg_min = s
        return pt_min, dist_min, seg_min
    else:
        p_start = np.array([path.poses[seg].pose.position.x, path.poses[seg].pose.position.y])
        p_end = np.array([path.poses[seg+1].pose.position.x, path.poses[seg+1].pose.position.y])
        v = p_end - p_start
        length_seg = np.linalg.norm(v)
        if length_seg == 0:
            return p_start, np.linalg.norm(x - p_start), seg
        v = v / length_seg
        dist_projected = np.dot(x - p_start, v)
        if dist_projected < 0.:
            pt_min = p_start
        elif dist_projected > length_seg:
            pt_min = p_end
        else:
            pt_min = p_start + dist_projected * v
        dist_min = np.linalg.norm(pt_min - x)
        return pt_min, dist_min, seg

def pure_pursuit_subgoal(path, pose, lookahead=2.0):
    """
    Compute the subgoal (to be used as cnn_goal) given:
      - path: a nav_msgs/Path message (global plan)
      - pose: the current robot pose (PoseStamped)
      - lookahead: the lookahead distance.
    Returns:
      - goal_local: a 2-element numpy array representing the subgoal coordinates in the robot's local frame.
    """
    # Extract the robot's position and orientation from the pose
    x = np.array([pose.pose.position.x, pose.pose.position.y])
    quat = [pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w]
    (_, _, theta) = euler_from_quaternion(quat)

    # Find the closest point on the path to the robot's current position
    pt, dist, seg = find_closest_point(path, x)

    # Compute the goal point on the path based on lookahead distance
    if dist > lookahead:
        goal = pt
    else:
        seg_max = len(path.poses) - 2
        p_end = np.array([path.poses[seg+1].pose.position.x, path.poses[seg+1].pose.position.y])
        dist_end = np.linalg.norm(x - p_end)
        current_seg = seg
        while dist_end < lookahead and current_seg < seg_max:
            current_seg += 1
            p_end = np.array([path.poses[current_seg+1].pose.position.x, path.poses[current_seg+1].pose.position.y])
            dist_end = np.linalg.norm(x - p_end)
        if dist_end < lookahead:
            # Use the very end of the path if the lookahead circle contains the end
            goal = np.array([path.poses[seg_max+1].pose.position.x, path.poses[seg_max+1].pose.position.y])
        else:
            # Find intersection along the segment
            pt_seg, _, seg_used = find_closest_point(path, x, current_seg)
            p_start = np.array([path.poses[seg_used].pose.position.x, path.poses[seg_used].pose.position.y])
            p_end = np.array([path.poses[seg_used+1].pose.position.x, path.poses[seg_used+1].pose.position.y])
            v = p_end - p_start
            length_seg = np.linalg.norm(v)
            if length_seg != 0:
                v = v / length_seg
                dist_projected = np.dot(x - pt_seg, v)
                cross_norm = np.linalg.norm(np.cross(x - pt_seg, v))
                # Calculate how far along the segment the intersection occurs
                goal = pt_seg + (np.sqrt(lookahead**2 - cross_norm**2) + dist_projected) * v
            else:
                goal = pt_seg

    # Transform the computed goal into the robot's local frame
    map_T_robot = np.array([[np.cos(theta), -np.sin(theta), x[0]],
                              [np.sin(theta),  np.cos(theta), x[1]],
                              [0,              0,             1]])
    inv_map_T_robot = np.linalg.inv(map_T_robot)
    goal_hom = np.array([goal[0], goal[1], 1])
    goal_local = np.matmul(inv_map_T_robot, goal_hom)
    return goal_local[0:2]

def create_path(poses):
    """
    Create a nav_msgs/Path message from a list of (x, y) tuples.
    """
    path = Path()
    path.poses = []
    for (x, y) in poses:
        ps = PoseStamped()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        # Set orientation to identity quaternion (0,0,0,1)
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path

def create_pose(x, y, theta):
    """
    Create a PoseStamped message with the given x, y, and theta (in radians).
    The orientation is computed from theta.
    """
    ps = PoseStamped()
    ps.pose.position.x = x
    ps.pose.position.y = y
    # Convert theta to a quaternion (assuming 2D: roll=pitch=0)
    cos_half = math.cos(theta / 2.0)
    sin_half = math.sin(theta / 2.0)
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = sin_half
    ps.pose.orientation.w = cos_half
    return ps

class TestPurePursuit(unittest.TestCase):
    def test_find_closest_point(self):
        # Create a simple path along the x-axis: (0,0), (1,0), (2,0), (3,0)
        poses = [(0, 0), (1, 0), (2, 0), (3, 0)]
        path = create_path(poses)
        # Test with a point near the middle of the second segment: (1.5, 0.2)
        x_point = np.array([1.5, 0.2])
        pt, dist, seg = find_closest_point(path, x_point)
        # For segment from (1,0) to (2,0), the projection should be (1.5, 0)
        expected_pt = np.array([1.5, 0.0])
        expected_dist = np.linalg.norm(x_point - expected_pt)
        self.assertTrue(np.allclose(pt, expected_pt, atol=1e-2),
                        f"Expected closest point {expected_pt}, got {pt}")
        self.assertAlmostEqual(dist, expected_dist, places=2,
                               msg=f"Expected distance {expected_dist}, got {dist}")
        self.assertEqual(seg, 1, f"Expected segment index 1, got {seg}")

    def test_pure_pursuit_subgoal(self):
        # Create a simple path along the x-axis: (0,0), (1,0), (2,0), (3,0)
        poses = [(0, 0), (1, 0), (2, 0), (3, 0)]
        path = create_path(poses)
        # Create a robot pose at (1.5, 0.2) facing east (theta=0)
        robot_pose = create_pose(1.5, 0.2, 0.0)
        # Compute the subgoal using a lookahead distance of 2.0
        subgoal = pure_pursuit_subgoal(path, robot_pose, lookahead=2.0)
        # Expected behavior:
        #   - Global subgoal computed from the path is (3, 0)
        #   - Transformed into the robot’s local frame (robot at (1.5,0.2) with theta=0)
        #     it becomes (3-1.5, 0-0.2) = (1.5, -0.2)
        expected_subgoal = np.array([1.5, -0.2])
        self.assertTrue(np.allclose(subgoal, expected_subgoal, atol=1e-2),
                        f"Expected subgoal {expected_subgoal}, got {subgoal}")

if __name__ == '__main__':
    unittest.main()
