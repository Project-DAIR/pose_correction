import rospy
# ROS Image message
from stag_ros.msg import STagMarkerArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import cv2
import numpy as np
import statistics
import tf

class PoseCorrection:

    def quaternion_rotation_matrix(self, Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02, 0],
                            [r10, r11, r12, 0],
                            [r20, r21, r22, 0],
                            [0, 0, 0, 1]])
                                
        return rot_matrix

    def __init__(self):
        self.window_size = 20

        self.sub = rospy.Subscriber("/stag_ros/markers", STagMarkerArray, self.callback, queue_size=self.window_size)
        self.pubX = rospy.Publisher("/corrected_pose/X", Float64, queue_size=self.window_size)
        self.pubY = rospy.Publisher("/corrected_pose/Y", Float64, queue_size=self.window_size)
        self.pubZ = rospy.Publisher("/corrected_pose/Z", Float64, queue_size=self.window_size)
        self.correct_pose = rospy.Publisher("/marker", Point, queue_size=self.window_size)

        self.x_arr = []
        self.y_arr = []
        self.z_arr = []
        self.x_avg = 0
        self.y_avg = 0
        self.z_avg = 0

        self.x_median = 0
        self.y_median = 0
        self.z_median = 0


    def callback(self, sTagMarkerArray):
        stag = sTagMarkerArray.stag_array[0]
        pose = stag.pose

        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z

        self.x_arr.append(pos_x)
        self.y_arr.append(pos_y)
        self.z_arr.append(pos_z)

        if self.window_size == len(self.x_arr) or self.window_size == len(self.y_arr) or self.window_size == len(self.z_arr):
            self.x_avg = sum(self.x_arr) / self.window_size
            self.y_avg = sum(self.y_arr) / self.window_size
            self.z_avg = sum(self.z_arr) / self.window_size


            point3d = Point()
            point3d.x = self.x_avg
            point3d.y = self.y_avg
            point3d.z = self.z_avg

            # median
            self.x_median = statistics.median(self.x_arr)
            self.y_median = statistics.median(self.y_arr)
            self.z_median = statistics.median(self.z_arr) 
            
            # point3d = Point()
            point3d.x = self.x_median
            point3d.y = -self.y_median
            point3d.z = self.z_median

            self.correct_pose.publish(point3d)
            
            # avg over window and reset
            # self.x_arr = []
            # self.y_arr = []
            # self.z_arr = []  

            # moving average
            self.x_arr.pop(0)
            self.y_arr.pop(0)
            self.z_arr.pop(0)



def main():
    rospy.init_node('pose_correction_node', anonymous= False)
    PoseCorrection()
    rospy.spin()

if __name__ == "__main__":
    main()