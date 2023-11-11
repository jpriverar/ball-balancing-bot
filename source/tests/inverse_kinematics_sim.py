import numpy as np
import matplotlib.pyplot as plt

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


# Base origin coordinates
origin1 = np.array([[0, 0, 0]]).T

# Platform radius
h1 = 6.0
h2 = 8.0

# Leg length
l1 = 4.5
l2 = 9

# Initial values of the desired efector's position and angle
offset = 8
x_angle = 0
y_angle = 0

# Get the base, active revolutive joint position (fixed)
a1 = np.array([[0, h1, 0, 1]]).T
a2 = np.array([[np.sqrt(3)*h1/2, -h1/2, 0, 1]]).T
a3 = np.array([[-np.sqrt(3)*h1/2, -h1/2, 0, 1]]).T

# To plot animation
fig = plt.figure()
#plt.ion()
ax = fig.add_subplot(1,1,1, projection='3d')

i = 0
while i < 100:
    #fig.canvas.clf()
    ax.cla()

    # Get the position of the spherical joint
    origin2 = np.array([[0, 0, offset]]).T

    # Points from the CS of origin 2 in homogenous coordinates
    b1_2 = np.array([[0, h2, 0, 1]]).T
    b2_2 = np.array([[np.sqrt(3)*h2/2, -h2/2, 0, 1]]).T
    b3_2 = np.array([[-np.sqrt(3)*h2/2, -h2/2, 0, 1]]).T

    # Points from the CS of origin 2
    b1 = np.dot(transformation_matrix_homogenous(np.radians(x_angle), np.radians(y_angle), 0, offset), b1_2)
    b2 = np.dot(transformation_matrix_homogenous(np.radians(x_angle), np.radians(y_angle), 0, offset), b2_2)
    b3 = np.dot(transformation_matrix_homogenous(np.radians(x_angle), np.radians(y_angle), 0, offset), b3_2)

    # print(a1.T)
    # print(a2.T)
    # print(a3.T)
    # print(b1.T)
    # print(b2.T)
    # print(b3.T)

    # Get revolute joint angles and passive revolute joint positions
    d1 = b1- a1
    gamma1 = np.arctan2(d1[2], d1[1])
    alpha1 = compute_alpha(l1, l2, d1)
    theta1 = compute_theta(l1, l2, alpha1, gamma1)
    j1 = a1 + np.array([[0, l1*np.cos(theta1[0]), l1*np.sin(theta1[0]), 0]]).T
    #print(d1)

    # Represent the points in a coplanar system.
    a2_tmp = np.dot(transformation_matrix_homogenous(0, 0, np.radians(120), 0), a2)
    b2_tmp = np.dot(transformation_matrix_homogenous(0, 0, np.radians(120), 0), b2) 
    d2 = b2_tmp - a2_tmp
    gamma2 = np.arctan2(d2[2], d2[1])
    alpha2 = compute_alpha(l1, l2, d2)
    theta2 = compute_theta(l1, l2, alpha2, gamma2)
    j2_tmp = a2_tmp + np.array([[0, l1*np.cos(theta2[0]), l1*np.sin(theta2[0]), 0]]).T
    j2 = np.dot(transformation_matrix_homogenous(0, 0, np.radians(-120), 0), j2_tmp) 
    
    # Represent the points in a coplanar system.
    a3_tmp = np.dot(transformation_matrix_homogenous(0, 0, np.radians(240), 0), a3)
    b3_tmp = np.dot(transformation_matrix_homogenous(0, 0, np.radians(240), 0), b3) 
    d3 = b3_tmp - a3_tmp
    gamma3 = np.arctan2(d3[2], d3[1])
    alpha3 = compute_alpha(l1, l2, d3)
    theta3 = compute_theta(l1, l2, alpha3, gamma3)
    j3_tmp = a2_tmp + np.array([[0, l1*np.cos(theta3[0]), l1*np.sin(theta3[0]), 0]]).T
    j3 = np.dot(transformation_matrix_homogenous(0, 0, np.radians(-240), 0), j3_tmp) 

    # print(f'd1:{d1[0]},{d1[1]}, gamma1:{np.degrees(gamma1)}, alpha1:{np.degrees(alpha1)}, theta1:{np.degrees(theta1)}, \
    #         d2:{d2[0]},{d2[1]}, gamma2:{np.degrees(gamma2)}, alpha2:{np.degrees(alpha2)}, theta2:{np.degrees(theta2)}')
    # print(f'l11:{np.linalg.norm(j1-a1):4f}, l12:{np.linalg.norm(b1-j1):4f} --------------- l21:{np.linalg.norm(j2-a2):4f}, l22:{np.linalg.norm(b2-j2):4f}')
    # print()

    # Plot both origins
    ax.plot_trisurf([a1[0,0], a2[0,0], a3[0,0]], [a1[1,0],a2[1,0], a3[1,0]], [a1[2,0], a2[2,0], a3[2,0]])
    ax.scatter(origin1[0], origin1[1], origin1[2], color='red', marker='o')
    ax.plot_trisurf([b1[0,0], b2[0,0], b3[0,0]], [b1[1,0], b2[1,0], b3[1,0]], [b1[2,0], b2[2,0], b3[2,0]])
    ax.scatter(origin2[0], origin2[1], origin2[2], color='green', marker='o')

    # Plot the arms
    ax.plot([a1[0], j1[0], b1[0]], [a1[1], j1[1], b1[1]], [a1[2], j1[2], b1[2]])
    ax.scatter([a1[0], j1[0], b1[0]], [a1[1], j1[1], b1[1]], [a1[2], j1[2], b1[2]], color='red', marker='o')

    ax.plot([a2[0], j2[0], b2[0]], [a2[1], j2[1], b2[1]], [a2[2], j2[2], b2[2]])
    ax.scatter([a2[0], j2[0], b2[0]], [a2[1], j2[1], b2[1]], [a2[2], j2[2], b2[2]], color='gray', marker='o')

    ax.plot([a3[0], j3[0], b3[0]], [a3[1], j3[1], b3[1]], [a3[2], j3[2], b3[2]])
    ax.scatter([a3[0], j3[0], b3[0]], [a3[1], j3[1], b3[1]], [a3[2], j3[2], b3[2]], color='green', marker='o')

    ax.set_ylim(-20, 20)
    ax.set_xlim(-20, 20)
    ax.set_zlim(0, 12)
    fig.canvas.draw()
    plt.pause(0.01)
    print(f'offset: {offset:<20}, x_angle: {x_angle:<20}, y_angle: {y_angle:<20}, theta_1: {np.degrees(theta1[0]):<20}, theta_2: {np.degrees(theta2[0]):<20}, theta_3: {np.degrees(theta3[0]):<20}')

    offset += 0.1
    #x_angle -= 0.25
    #y_angle += 0.05
    i += 1

plt.ioff()
plt.show()
