import h5py
import numpy as np
import os
import math
import cv2

import tf
import os
import cv2
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np
import argparse

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000

def calc(dis, azimuth, laser_id):
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth * ROTATION_RESOLUTION * np.pi / 180.0
    Y = -R * np.cos(omega) * np.sin(alpha)
    X = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    return [X, Y, Z, 0]

def write_pcb(points, path, WIDTH, i):
    
    path = path+'/training/pcb/'+str(i).zfill(6)+'.pcd'
    if os.path.exists(path):
        os.remove(path)

    out = open(path, 'a')
    # headers
    out.write('# .PCD v.7 - Point Cloud Data file format\nVERSION .7')
    out.write('\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1')
    string = '\nWIDTH ' + str(WIDTH)
    out.write(string)
    out.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
    string = '\nPOINTS ' + str(WIDTH)
    out.write(string)
    out.write('\nDATA ascii')

    #datas
    for i in range(WIDTH):
        string = '\n' + str(points[i][0]) + ' ' + str(points[i][1]) + ' ' \
                +str(points[i][2]) + ' ' + str(points[i][3])
        out.write(string)
    out.close()
    
def save_camera_data(bag, cv_image, camera_frame_id, time, topic):
    calib = CameraInfo()
    calib.header.frame_id = camera_frame_id
#     calib.width, calib.height = tuple(util['S_rect_{}'.format(camera_pad)].tolist())
#     calib.distortion_model = 'plumb_bob'
#     calib.K = util['K_{}'.format(camera_pad)]
#     calib.R = util['R_rect_{}'.format(camera_pad)]
#     calib.D = util['D_{}'.format(camera_pad)]
#     calib.P = util['P_rect_{}'.format(camera_pad)]

    calib.height, calib.width = cv_image.shape[:2]

    encoding = "bgr8"
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
    image_message.header.frame_id = camera_frame_id
    image_message.header.stamp = rospy.Time.from_sec(float(time))
    topic_ext = "/image_raw"
    
    calib.header.stamp = image_message.header.stamp
    
    bag.write(topic + topic_ext, image_message, t = image_message.header.stamp)
    bag.write(topic + '/camera_info', calib, t = calib.header.stamp) 
    
    
def save_velo_data(bag, velo, velo_frame_id, time, topic):
    print (len(velo), len(velo[0]))
    scan = (np.array(velo, dtype=np.float32))#.reshape(-1, 4)
    # create header
    header = Header()
    header.frame_id = velo_frame_id
    header.stamp = rospy.Time.from_sec(float(time))

    # fill pcl msg
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('i', 12, PointField.FLOAT32, 1)]
    pcl_msg = pcl2.create_cloud(header, fields, scan)

    bag.write(topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)

def save_imu_data(bag, roll, pitch, yaw, imu_frame_id, time, topic):
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    imu = Imu()
    imu.header.frame_id = imu_frame_id
    imu.header.stamp = rospy.Time.from_sec(time)
    imu.orientation.x = q[0]
    imu.orientation.y = q[1]
    imu.orientation.z = q[2]
    imu.orientation.w = q[3]
    bag.write(topic, imu, t=imu.header.stamp)
    
def save_gps_fix_data(bag, x, y, gps_frame_id, time, topic):
    navsatfix_msg = NavSatFix()
    navsatfix_msg.header.frame_id = gps_frame_id
    navsatfix_msg.header.stamp = rospy.Time.from_sec(time)
    navsatfix_msg.latitude = y
    navsatfix_msg.longitude = x
    navsatfix_msg.status.service = 1
    bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


root_dir = "/home/hywel/DataDisk/lidar-image-gps-ahrs"
for filename in os.listdir(root_dir):
    print (filename)
    path_dir = os.path.join(root_dir, filename)
    for name in ['calib', 'image_2', 'label_2', 'velodyne', 'pcb']:
        tmp = os.path.join(path_dir,'training',name)
        if not os.path.exists(tmp):
            os.makedirs(tmp)
    
    image_file_name = os.path.join(root_dir, filename, filename+'.avi')
    h5_file_name = os.path.join(root_dir, filename, filename+'.h5')
    
    try:
        f = h5py.File(h5_file_name, 'r')
    except:
        print ("can't open ", h5_file_name)
        continue

    lidar_data = f['lidar_data']
    extra_data = f['extra_data']
    timestamp = f['timestamp']
    #[posx,posy,roll,pitch,yaw,posx102,posy102,self.runtime]

    cap = cv2.VideoCapture(image_file_name)

    # frames = []
    # while(1):
    #     # get a frame
    #     ret, frame = cap.read()
    #     if ret:
    #         frames.append(frame)
    #     else:
    #         break
    # print ("image frames: ",len(frames))
    bridge = CvBridge()
    compression = rosbag.Compression.NONE
    bag = rosbag.Bag("kitti_{}.bag".format(filename), 'w', compression=compression)

    num_old = -1
    for i, num in enumerate(range(0,len(lidar_data), 20)):
        print ("record frame: ", num)

        for _ in range(num_old,num):
            ret, frame = cap.read()
            if not ret:
                print ("lack vidio frame")
                break
        num_old = num

        points = []
        one_lidar = lidar_data[num,:]
        assert len(one_lidar)//75==408
        WIDTH = 75*24*16
        for j in range(75):
            tmp_data = one_lidar[408*j:408*(j+1)]
            for k in range(24):
                dis_list = tmp_data[16*k:16*(k+1)]
                azimuth = tmp_data[384+k]
                for w in range(16):
                    points.append(calc(dis_list[w], azimuth, w))

        print (len(points))
        posx,posy,roll,pitch,yaw,posx102,posy102,runtime = extra_data[num,:]
        
        save_camera_data(bag, frame, "camera", i, "pointgrey")
        save_velo_data(bag, points, "lidar", i, "velo")
        save_imu_data(bag, roll, pitch, yaw, "imu", i, "imu")
        save_gps_fix_data(bag, posx,posy, "gps", i, "gps")
        
        # write_pcb(points, path_dir, WIDTH, i)
        # name = path_dir + "/training/image_2/"+str(i).zfill(6)+".png"
        # cv2.imwrite(name, frame)
    print("## OVERVIEW ##")
    print(bag)
    bag.close()
