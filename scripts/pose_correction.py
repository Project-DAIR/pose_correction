import rospy

# ROS Image message
from stag_ros.msg import STagMarkerArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

import statistics
import tf
import sys


class PoseCorrection:
    def __init__(self, filter_type):
        self.window_size = 8

        self.sub = rospy.Subscriber(
            "/stag_ros/markers",
            STagMarkerArray,
            self.callback,
            queue_size=self.window_size,
        )
        self.smooth_pose = rospy.Publisher(
            "/marker", Point, queue_size=self.window_size
        )

        self.x_arr = []
        self.y_arr = []
        self.z_arr = []

        self.point3d = Point()

        self.filter_type = filter_type

    def callback(self, sTagMarkerArray):
        stag = sTagMarkerArray.stag_array[0]
        pose = stag.pose

        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z

        self.x_arr.append(pos_x)
        self.y_arr.append(pos_y)
        self.z_arr.append(pos_z)

        if (
            self.window_size == len(self.x_arr)
            or self.window_size == len(self.y_arr)
            or self.window_size == len(self.z_arr)
        ):

            if filter_type == "moving_avg_filter":
                # moving average
                x_avg = sum(self.x_arr) / self.window_size
                y_avg = sum(self.y_arr) / self.window_size
                z_avg = sum(self.z_arr) / self.window_size

                self.point3d.x = x_avg
                self.point3d.y = -y_avg
                self.point3d.z = z_avg

                self.x_arr.pop(0)
                self.y_arr.pop(0)
                self.z_arr.pop(0)

            elif filter_type == "median_avg_filter":

                # median filter
                x_median = statistics.median(self.x_arr)
                y_median = statistics.median(self.y_arr)
                z_median = statistics.median(self.z_arr)

                self.point3d.x = x_median
                self.point3d.y = -y_median
                self.point3d.z = z_median

            elif filter_type == "avg_filter":

                # avg over window and reset
                x_avg = sum(self.x_arr) / self.window_size
                y_avg = sum(self.y_arr) / self.window_size
                z_avg = sum(self.z_arr) / self.window_size

                self.point3d.x = x_avg
                self.point3d.y = -y_avg
                self.point3d.z = z_avg

                self.x_arr = []
                self.y_arr = []
                self.z_arr = []

            print("Filter type : ", filter_type)
            print(self.point3d)
            print()
            self.smooth_pose.publish(self.point3d)


def main(filter_type):
    rospy.init_node("pose_correction_node", anonymous=False)
    PoseCorrection(filter_type)
    rospy.spin()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) == 2:
        filter_type = args[1]
    else:
        filter_type = "moving_avg_filter"
    main(filter_type)
