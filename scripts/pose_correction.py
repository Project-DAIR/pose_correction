import statistics
import sys

import rospy
import tf
from comm_pipeline.srv import (ActivateStag, ActivateStagResponse, FoundMarker,
                               FoundMarkerRequest, GetTarget,
                               GetTargetResponse)
from geometry_msgs.msg import Point
from stag_ros.msg import STagMarkerArray


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
            "~smooth_pose", Point, queue_size=self.window_size
        )

        self.x_arr = []
        self.y_arr = []
        self.z_arr = []

        self.point3d = Point()

        self.filter_type = filter_type

        self.get_target_server = rospy.Service(
            '~get_target', GetTarget, self.get_target_callback)
        self.activate_stag_server = rospy.Service(
            '~activate_stag', ActivateStag, self.activate_stag_callback)
        self.filter_initialized = False
        self.is_activated = False

        self.found_marker_client = rospy.ServiceProxy(
            "/planner/found_marker", FoundMarker)

        self.last_seen = rospy.Time.now()
        self.tracking_timeout = 1

        self.br = tf.TransformBroadcaster()

    def callback(self, sTagMarkerArray):
        if not self.is_activated:
            return

        stag = sTagMarkerArray.stag_array[0]
        pose = stag.pose

        pos_x = pose.position.x
        pos_y = pose.position.y
        pos_z = pose.position.z

        self.x_arr.append(pos_x)
        self.y_arr.append(pos_y)
        self.z_arr.append(pos_z)

        self.last_seen = rospy.Time.now()

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

            # Send transform (helps with visualization)
            self.br.sendTransform((self.point3d.x, -self.point3d.y, self.point3d.z), (stag.pose.orientation.x, stag.pose.orientation.y, stag.pose.orientation.z, stag.pose.orientation.w),
                                  rospy.Time.now(),
                                  "filter",
                                  "oak_left_camera_optical_frame")

            self.smooth_pose.publish(self.point3d)

            if (not self.filter_initialized):
                try:
                    req = FoundMarkerRequest()
                    req.position.point = self.point3d
                    req.position.header.stamp = rospy.Time.now()

                    # Make sure service is available beefore we call it
                    rospy.wait_for_service('/planner/found_marker')
                    res = self.found_marker_client(req)

                    if res.success:
                        print("Found marker service called succesfully")
                    else:
                        print("ERROR: Something went wrong...")

                except rospy.ServiceException as ex:
                    print("Service did not process request: " + str(ex))

            self.filter_initialized = True

    def get_target_callback(self, req):
        res = GetTargetResponse()

        # Havent gotten good tracking on marker
        if not self.filter_initialized:
            print("No marker pose")
            res.isTracked = False
            return res

        res.position = self.point3d

        if (rospy.Time.now() - self.last_seen) > rospy.Duration.from_sec(self.tracking_timeout):
            res.isTracked = False
            self.filter_initialized = False
            self.x_arr = []
            self.y_arr = []
            self.z_arr = []
        else:
            res.isTracked = True

        return res

    def activate_stag_callback(self, req):

        self.is_activated = True
        res = ActivateStagResponse(True)

        return res


def main(filter_type):
    rospy.init_node("marker", anonymous=False)
    PoseCorrection(filter_type)
    rospy.spin()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) == 2:
        filter_type = args[1]
    else:
        filter_type = "moving_avg_filter"
    main(filter_type)
