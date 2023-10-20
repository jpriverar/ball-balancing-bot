import numpy as np
import matplotlib.pyplot as plt

def transformation_matrix_homogenous(th1, th2, offset):
    return np.array([[np.cos(th2), 0, np.sin(th2), 0],
                     [np.sin(th1)*np.sin(th2), np.cos(th1), -np.cos(th2)*np.sin(th1), 0],
                     [-np.cos(th1)*np.sin(th2), np.sin(th1), np.cos(th1)*np.sin(th2), offset],
                     [0,0,0,1]])


def transformation_matrix_2d(th):
    return np.array([[np.cos(th), -np.sin(th)],
                     [np.sin(th), np.cos(th)]])


def compute_alpha(l1, l2, d):
    cos_alpha = (np.linalg.norm(d)**2 - l1**2 - l2**2)/(2*l1*l2)
    sin_alpha = np.sqrt(1-(cos_alpha**2))
    return np.arctan2(sin_alpha, cos_alpha)


def compute_theta(l1, l2, alpha, gamma):
    return np.pi - gamma - np.arctan2(l2*np.sin(alpha), l1 + l2*np.cos(alpha))


# Base origin coordinates
origin1 = np.array([[0],[0]])

# Platform radius
h1 = 10
h2 = 8

# Leg length
l1 = 6
l2 = 8

# Initial values of the desired efector's position and angle
offset = 10
angle = 0

# Get the base, active revolutive joint position (fixed)
a1 = np.array([[-h1], [0]])
a2 = np.array([[h1], [0]])

# To plot animation
plt.figure()
plt.ion()

i = 0
while i < 100:
    plt.clf()

    # Get the position of the spherical joint
    origin2 = np.array([[0], [offset]])
    b1 = np.dot(transformation_matrix_2d(np.radians(angle)), np.array([[-h2], [0]])) + origin2
    b2 = np.dot(transformation_matrix_2d(np.radians(angle)), np.array([[h2], [0]])) + origin2

    # Get revolute joint angles
    d1 = b1 - a1
    gamma1 = np.arctan2(d1[1], d1[0])
    alpha1 = compute_alpha(l1, l2, d1)
    theta1 = compute_theta(l1, l2, alpha1, gamma1)

    d2 = b2 - a2
    d2[0] *= -1
    gamma2 = np.arctan2(d2[1], d2[0])
    alpha2 = compute_alpha(l1, l2, d2)
    theta2 = compute_theta(l1, l2, alpha2, gamma2)
    print(f'd1:{d1[0]},{d1[1]}, gamma1:{np.degrees(gamma1)}, alpha1:{np.degrees(alpha1)}, theta1:{np.degrees(theta1)}, \
           d2:{d2[0]},{d2[1]}, gamma2:{np.degrees(gamma2)}, alpha2:{np.degrees(alpha2)}, theta2:{np.degrees(theta2)}')

    # Get the position of the passive revolute joint
    j1 = a1 + np.array([-l1*np.cos(theta1), l1*np.sin(theta1)])
    j2 = a2 + np.array([l1*np.cos(theta2), l1*np.sin(theta2)])
    print(f'l11:{np.linalg.norm(j1-a1):4f}, l12:{np.linalg.norm(b1-j1):4f} --------------- l21:{np.linalg.norm(j2-a2):4f}, l22:{np.linalg.norm(b2-j2):4f}')
    print()

    # Plot both origins
    plt.plot([a1[0], a2[0]], [a1[1], a2[1]])
    plt.scatter(origin1[0], origin1[1], color='red', marker='o')
    plt.plot([b1[0], b2[0]], [b1[1], b2[1]])
    plt.scatter(origin2[0], origin2[1], color='green', marker='o')

    # Plot left arm
    plt.plot([a1[0], j1[0], b1[0]], [a1[1], j1[1], b1[1]])
    plt.scatter([a1[0], j1[0], b1[0]], [a1[1], j1[1], b1[1]], color='gray', marker='o')

    # Plot right arm
    plt.plot([a2[0], j2[0], b2[0]], [a2[1], j2[1], b2[1]])
    plt.scatter([a2[0], j2[0], b2[0]], [a2[1], j2[1], b2[1]], color='gray', marker='o')

    plt.ylim(0, 12)
    plt.xlim(-20, 20)
    plt.draw()
    plt.pause(0.01)

    offset -= 0.05
    #angle += 0.1
    i += 1

plt.ioff()
plt.show()
