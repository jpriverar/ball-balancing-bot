import numpy as np

def x_rot(th):
    return np.array([[1,          0,           0],
                     [0, np.cos(th), -np.sin(th)],
                     [0, np.sin(th),  np.cos(th)]])


def y_rot(th):
    return np.array([[np.cos(th),  0, np.sin(th)],
                     [0,           1,          0],
                     [-np.sin(th), 0, np.cos(th)]])


def z_rot(th):
    return np.array([[np.cos(th), -np.sin(th), 0],
                     [np.sin(th),  np.cos(th), 0],
                     [0,           0,          1]])


def transformation_matrix_homogenous(x_th, y_th, z_th, offset):
    combined_rot = z_rot(z_th).dot(y_rot(y_th).dot(x_rot(x_th)))
    traslation_vec = np.array([[0, 0, offset]]).T
    homogenous_vec = np.array([[0, 0, 0, 1]])
    return np.concatenate((np.concatenate((combined_rot, traslation_vec), axis=1), homogenous_vec), axis=0)


def transformation_matrix_2d(th):
    return np.array([[np.cos(th), -np.sin(th)],
                     [np.sin(th), np.cos(th)]])


def compute_alpha(l1, l2, d):
    cos_alpha = (np.linalg.norm(d)**2 - l1**2 - l2**2)/(2*l1*l2)
    sin_alpha = np.sqrt(1-(cos_alpha**2))
    return np.arctan2(sin_alpha, cos_alpha)


def compute_theta(l1, l2, alpha, gamma):
    return np.pi - gamma - np.arctan2(l2*np.sin(alpha), l1 + l2*np.cos(alpha))