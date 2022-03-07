#import visualization_msgs.msg as visualization_msgs
#import geometry_msgs.msg as geometry_msgs
#import nav_msgs.msg as nav_msgs
import numpy as np

RED = [1.0, 0.0, 0.0]
BLUE = [0.0, 0.0, 1.0]
GREEN = [0.0, 1.0, 0.0]
YELLOW = [1.0, 1.0, 0.0]
LIGHTBLUE = [0.0, 1.0, 1.0]

TRAVERSABLE = [0.0, 0.1, 0.0]
COVERABLE = [0.0, 0.3, 0.0]
PATH = [0.0, 1.0, 1.0]

def point_marker(id, stamp, point, color, ns):
    msg = visualization_msgs.Marker()
    msg.header.frame_id = "my_frame"
    msg.header.stamp = stamp
    msg.ns = ns
    msg.id = id
    msg.type = 1
    msg.action = 0
    msg.pose = geometry_msgs.Pose()
    msg.pose.position.x = point[0]
    msg.pose.position.y = point[1]
    msg.pose.position.z = point[2]
    msg.scale.x = 0.5
    msg.scale.y = 0.5
    msg.scale.z = 0.5
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]
    msg.color.a = 1.0
    msg.frame_locked = True
    return msg

def arrow(id, stamp, start_point, direction, color): 
    msg = visualization_msgs.Marker()
    msg.header.frame_id = "my_frame"
    msg.header.stamp = stamp
    msg.id = id
    msg.type = 0
    msg.action = 0
    msg.pose = geometry_msgs.Pose()
    msg.pose.position.x = start_point[0]
    msg.pose.position.y = start_point[1]
    msg.pose.position.z = start_point[2]
    
    #calculating the half-way vector.
    u = [1,0,0]
    norm = np.linalg.norm(direction)
    v = np.asarray(direction)/norm 
    if (np.array_equal(u, v)):
        msg.pose.orientation.w = 1
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
    elif (np.array_equal(u, np.negative(v))):
        msg.pose.orientation.w = 0
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 1
    else:
        half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
        msg.pose.orientation.w = np.dot(u, half)
        temp = np.cross(u, half)
        msg.pose.orientation.x = temp[0]
        msg.pose.orientation.y = temp[1]
        msg.pose.orientation.z = temp[2]
    norm = np.math.sqrt(msg.pose.orientation.x*msg.pose.orientation.x + msg.pose.orientation.y*msg.pose.orientation.y + 
        msg.pose.orientation.z*msg.pose.orientation.z + msg.pose.orientation.w*msg.pose.orientation.w)
    if norm == 0:
        norm = 1
    msg.pose.orientation.x /= norm
    msg.pose.orientation.y /= norm
    msg.pose.orientation.z /= norm
    msg.pose.orientation.w /= norm
    msg.scale.x = 1.0
    msg.scale.y = 0.1
    msg.scale.z = 0.1
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]
    msg.color.a = 1.0
    msg.frame_locked = True
    return msg

def line_marker(id, stamp, path, color, ns):
    msg = visualization_msgs.Marker()
    msg.header.frame_id = "my_frame"
    msg.header.stamp = stamp
    msg.ns = ns
    msg.id = id
    msg.type = 4
    msg.action = 0
    msg.points = []
    for point in path:
        position = geometry_msgs.Point()
        position.x = point[0]
        position.y = point[1]
        position.z = point[2]+0.05
        msg.points.append(position)
    #msg.points = path
    msg.scale.x = 0.05
    msg.scale.y = 0.05
    msg.scale.z = 0.05
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]
    msg.color.a = 1.0
    msg.frame_locked = True
    return msg


def path(waypoints):
    msg = nav_msgs.Path()
    msg.header.frame_id = "my_frame"
    for point in waypoints:
        pose_stamped = geometry_msgs.PoseStamped()
        pose = geometry_msgs.Pose()
        pose.position = geometry_msgs.Point()
        pose.position.x = point[0]
        pose.position.y = point[1]
        pose.position.z = point[2]

        pose_stamped.pose = pose
        msg.poses.append(pose_stamped)
    return msg