import numpy as np
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# ==================================================================================================================

def compute_box_3d(dim, location, rotation_y):
  # dim: 3
  # location: 3
  # rotation_y: 1
  # return: 8 x 3
  c, s = np.cos(rotation_y), np.sin(rotation_y)
  R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)
  l, w, h = dim[2], dim[1], dim[0]
  x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
  y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
  z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]

  corners_3d_cam2 = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
  corners_3d_cam2[0,:] += location[0]
  corners_3d_cam2[1,:] += location[1]
  corners_3d_cam2[2,:] += location[2]

  return corners_3d_cam2

# ==================================================================================================================

def color_select(cls, marker):

    if cls == 'Car':
      marker.color.r = 0      # Green
      marker.color.g = 1
      marker.color.b = 0

    elif cls == 'Pedestrian':
      marker.color.r = 1      # Red
      marker.color.g = 0
      marker.color.b = 0

    elif cls == 'Cyclist':
      marker.color.r = 1      # Yellow
      marker.color.g = 1
      marker.color.b = 0

    elif cls == 'Truck':
      marker.color.r = 0      # Cyan
      marker.color.g = 1
      marker.color.b = 1

    elif cls == 'Van':
      marker.color.r = 1      # Purple
      marker.color.g = 0
      marker.color.b = 1

    else:
      marker.color.r = 1      # White
      marker.color.g = 1
      marker.color.b = 1

    return marker

# ==================================================================================================================

lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6],
         [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]

def box_to_marker(ob, cls, i):

  detect_points_set = []
  for x in range(8):
    detect_points_set.append(Point(ob[x][0], ob[x][1], ob[x][2]))

  marker = Marker()
  marker.header.frame_id = 'map'
  marker.header.stamp = rospy.Time.now()
  marker.id = i
  marker.action = Marker.ADD
  marker.type = Marker.LINE_LIST
  marker.lifetime = rospy.Duration(0)

  marker = color_select(cls, marker)
  marker.color.a = 1
  marker.scale.x = 0.2
  marker.points = []

  for line in lines:
    marker.points.append(detect_points_set[line[0]])
    marker.points.append(detect_points_set[line[1]])

  return marker

