import math
from shapely import Point
from geometry_msgs.msg import Pose, Point as RosPoint # type: ignore

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def ros_point_to_shapely_point(ros_point):
    if not isinstance(ros_point, Point) and not isinstance(ros_point, RosPoint):
        ros_point = Point(ros_point[0], ros_point[1])
    return Point(ros_point.x, ros_point.y)

def ros_quaternion_to_euler(ros_quaternion):
    r, p, y = euler_from_quaternion(ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w)
    return math.degrees(y)

def get_cell_id_in_dict_from_continuous_position(grid_dict, position):
    if not isinstance(position, Point):
        position = Point(position)

    if isinstance(grid_dict, dict):
        for id, polygon in grid_dict.items():
            if polygon.buffer(0.1).contains(position):
                return id
    return None