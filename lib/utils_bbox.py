import cv2
import numpy as np
from .utils_pointcloud import compute_box_3d

# ==================================================================================================================

def drawing(img, cls, dim, loc, rot_y, calib):

  box_3d = compute_box_3d(dim, loc, rot_y)
  box_2d = project_to_image(box_3d.transpose(1, 0), calib)
  color = color_select(cls)
  img = draw_box_3d(img, box_2d, color)

  return img

# ==================================================================================================================

def project_to_image(pts_3d, P):
  # pts_3d: n x 3
  # P: 3 x 4
  # return: n x 2
  pts_3d_homo = np.concatenate([pts_3d, np.ones((pts_3d.shape[0], 1))], axis=1)
  pts_2d = np.dot(P, pts_3d_homo.transpose(1, 0)).transpose(1, 0)
  pts_2d = pts_2d[:, :2] / pts_2d[:, 2:]

  return pts_2d

# ==================================================================================================================

def color_select(cls):

    if cls == 'Car':
      color = (0, 255, 0)     # Green

    elif cls == 'Pedestrian':
      color = (0, 0, 255)     # Red

    elif cls == 'Cyclist':
      color = (0, 255, 255)   # Yellow

    elif cls == 'Truck':
      color = (255, 255, 0)   # Cyan

    elif cls == 'Van':
      color = (255, 0, 255)   # Purple

    else:
      color = (255, 255, 255) # White

    return color

# ==================================================================================================================

face_idx = [[0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7]]

def draw_box_3d(image, corners, c):

  for ind_f in range(3, -1, -1):
    f = face_idx[ind_f]
    for j in range(4):
      cv2.line(image, (int(corners[f[j],       0]), int(corners[f[j],       1]) ),
                      (int(corners[f[(j+1)%4], 0]), int(corners[f[(j+1)%4], 1]) ), c, 2, lineType=cv2.LINE_AA)
    if ind_f == 0:
      cv2.line(image, (int(corners[f[0], 0]), int(corners[f[0], 1]) ),
                      (int(corners[f[2], 0]), int(corners[f[2], 1]) ), c, 1, lineType=cv2.LINE_AA)
      cv2.line(image, (int(corners[f[1], 0]), int(corners[f[1], 1]) ),
                      (int(corners[f[3], 0]), int(corners[f[3], 1]) ), c, 1, lineType=cv2.LINE_AA)
  return image

