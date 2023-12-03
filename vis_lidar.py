import rospy
import numpy as np

from lib.utils_pointcloud import compute_box_3d, box_to_marker
from lib.kitti_utils import Calibration

import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

marker_array = MarkerArray()

# ==================================================================================================================

header = std_msgs.msg.Header()
header.frame_id = 'map'

fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]

pointcloud_pub = rospy.Publisher('/pointcloud',   PointCloud2, queue_size=10)
marker_pub     = rospy.Publisher('/detect_box3d', MarkerArray, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1000)

# ==================================================================================================================

name = '000010'

while not rospy.is_shutdown():

  # loading pointcloud
  scan = np.fromfile('/data_1TB_1/kitti/training/velodyne/' + name + '.bin', dtype=np.float32).reshape((-1, 4))
  pointcloud_msg = pcl2.create_cloud(header, fields, scan)

  # loading calib
  calib = Calibration('/data_1TB_1/kitti/training/calib/' + name + '.txt', from_video=False)

  # loading label
  f = open('/data_1TB_1/kitti/training/label_2/' + name + '.txt', 'r')
  for i, line in enumerate(f.readlines()):
    line_str = line.strip().split(' ')
    cls = line_str[0]
    dim   = np.array(line_str[8:11], dtype=np.float32)
    loc   = np.array(line_str[11:14], dtype=np.float32)
    rot_y = np.array(line_str[14], dtype=np.float32)

    if cls!= 'DontCare':
      box_3d = compute_box_3d(dim, loc, rot_y)
      corner_3d_velo = calib.project_rect_to_velo(box_3d.T)  
      marker = box_to_marker(corner_3d_velo, cls, i)
      marker_array.markers.append(marker)

  marker_pub.publish(marker_array)
  pointcloud_pub.publish(pointcloud_msg)
  rate.sleep()

