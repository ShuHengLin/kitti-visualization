import cv2
import numpy as np
from lib.utils_bbox import drawing

# ==================================================================================================================

def visualization(name):

  img = cv2.imread('/data_1TB_1/kitti/training/image_2/' + name + '.png')

  # loading calib
  calib = np.zeros((3, 4))
  f = open('/data_1TB_1/kitti/training/calib/' + name + '.txt', 'r')
  for line in f.readlines():
    line_str = line.strip().split(' ')
    if line_str[0] == 'P2:':
      calib = np.array(line_str[1:], dtype=np.float32).reshape(3, 4)

  # loading label
  f = open('/data_1TB_1/kitti/training/label_2/' + name + '.txt', 'r')
  for line in f.readlines():
    line_str = line.strip().split(' ')
    cls = line_str[0]
    dim   = np.array(line_str[8:11], dtype=np.float32)
    loc   = np.array(line_str[11:14], dtype=np.float32)
    rot_y = np.array(line_str[14], dtype=np.float32)

    if cls!= 'DontCare':
      img = drawing(img, cls, dim, loc, rot_y, calib)

  # cv2.imwrite(name + '_gt.png', img)
  cv2.imshow(name, img)
  cv2.waitKey(0)

# ==================================================================================================================

if __name__ == '__main__':

  name = '000010'
  visualization(name)

