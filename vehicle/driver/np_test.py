import numpy as np
from math import pi
dis_traveled = np.array([.0])
pose = np.array([1, 1, pi/4])

def pos_update(d_vect):
    global pose, dis_traveled
    # d_vect = np.array([0, 1, 0])
    # 位置变化矩阵
    d_pose_transform = np.array([[np.cos(pose[2]), np.sin(pose[2])], 
                                [-np.sin(pose[2]), np.cos(pose[2])]])
    d_pose_xy = np.dot(d_vect[:2], d_pose_transform)
    dis_traveled += np.sum(d_pose_xy**2, keepdims=True)**0.5
    # 
    d_pose = np.append(d_pose_xy, values=d_vect[2])
    # 更新坐标位置
    pose = pose + d_pose
    print(dis_traveled)
    # print(d_pose)
    print(pose)

def test_dot():
    ss = np.array([1, 2])
    dd = np.array([[3, 4], [5, 6]])
    print(np.dot(ss, dd))

def test_clip():
    np_data = np.array([1, 2, 3, 4, 5])
    print(np.clip(np_data, 2, 4))

def arg_test(arg1, arg2, arg3=None):
    print(arg1)
    print(arg2)
    print(arg3)

def np_test12(d_vect):
    # 位置变化矩阵
    import math
    pose = [.0,.0,.0]
    z_angle = pose[2]
    d_pose_transform = np.array([[math.cos(z_angle), math.sin(z_angle)], 
                                [-math.sin(z_angle), math.cos(z_angle)]])
    print("z_angle:", z_angle, "d_pose_transform:", d_pose_transform)
    # 车子坐标变化转为世界坐标变化
    d_pose_xy = np.dot(d_vect[:2], d_pose_transform)
    print("vect:",d_vect, "tans:",d_pose_xy)

def np_t_test():
    import math
    radius = 0.6
    # 根据小车三轮的运动计算小车运动， 正解
    transform_forward = np.array([[0, 2/3, 1/radius/3],
                                        [-1/2/math.cos(math.pi/6), -1/3, 1/radius/3],
                                        [1/2/math.cos(math.pi/6), -1/3, 1/radius/3]])
    print("transform_forward:", transform_forward)
    # 根据小车运动计算小车三轮的运动， 逆解
    transform_inverse = np.array([[0, -math.cos(math.pi/6), math.cos(math.pi/6)],
                                        [1, -math.sin(math.pi/6), -math.sin(math.pi/6)],
                                        [radius, radius, radius]])
    print("transform_forward:", transform_inverse)
    # 获取矩阵的逆
    transform_inverse = np.linalg.inv(transform_forward)
    # transform_forward = transform_inverse
    print("transform_forward:", transform_forward)

def np_t_test():
    rx = 0.3
    ry = 0.4
    r = rx + ry
    transform_forward = np.array([[1/4, 1/4, 1/r/4],
                                 [-1/4, 1/4, 1/r/4],
                                 [-1/4, -1/4, 1/r/4],
                                 [1/4, -1/4, 1/r/4]])
    transform_inverse = np.array([[1, -1, -1, 1],
                                  [1, 1, -1, -1],
                                  [r, r, r, r]])

    ret = np.dot(transform_inverse, transform_forward)
    print("ret:", ret)
    
    # print("transform_inverse:", transform_inverse)
    # 获取矩阵的逆
    # transform_inverse = np.linalg.inv(transform_inverse)
    # print("transform_inverse:", transform_inverse)

def np_test12():
    r = 0.2
    # 根据小车差速二轮运动计算小车运动， 正解
    transform_forward = np.array([[1/2, 0, 1/r],
                                  [1/2, 0, -1/r]])
    
    # 根据小车运动计算小车差速二轮运动， 逆解
    transform_inverse = np.array([[  1,    1],
                                  [  0,    0],
                                  [r/2, -r/2]])
    ret = np.dot(transform_inverse, transform_forward)
    print("ret:", ret)
    print("transform_inverse:", transform_inverse)
    print("transform_inverse:", transform_forward)
# args_list = {'arg1': 1, 'arg2': 2}
# arg_test(**args_list, arg3=3)
# arg_test(1, 2)
# arg_test(1, 2, 3)
# arg_test(1, 2, 3, 4)
# arg_test(1, 2, 3, 4, 5)
# np_test12(np.array([0.01, 0, 0]))
np_test12()
# test_dot()
# test_clip()
# for i in range(10):
#     pos_update([0.1, 0, 0.1])

# d_pose = d_vect[0:2], np.sin(pose[2]) + d_vect[1] * np.cos(pose[2])
# print(d_pose)