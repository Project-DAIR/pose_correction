import rospy

from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import statistics

class LidarCorrection:
    def __init__(self):
        self.window_size = 32
        
        self.sub = rospy.Subscriber(
            "raw_lidar",
            LaserScan,
            self.callback,
            queue_size=self.window_size,
        )

        self.smooth_lidar = rospy.Publisher(
            "~smooth_lidar", Int16, queue_size=self.window_size
        )

        self.d_arr = []
        self.d_moving_avg = -1
        self.tracking_timeout = 1
        self.last_seen = rospy.Time.now()

    def callback(self, raw_lidar):
        distance_in_cm = raw_lidar.ranges[0]
        self.d_arr.append(distance_in_cm)

        self.last_seen = rospy.Time.now()

        if self.window_size == len(self.d_arr):
            self.d_moving_avg = round(statistics.median(self.d_arr))
            # self.d_arr = []

            #self.d_moving_avg = round(sum(self.d_arr) / self.window_size)
            self.d_arr.pop(0)

            # print(self.d_moving_avg)
            self.smooth_lidar.publish(self.d_moving_avg)

            
def main():
    rospy.init_node("lidar_correction", anonymous=False)
    LidarCorrection()
    rospy.spin()

if __name__ == "__main__":
    main()
