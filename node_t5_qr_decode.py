#!/usr/bin/env python
"""This python script detect which package is what color using qr decoding"""

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pkg_task5.srv import sendColoursPackages, sendColoursPackagesResponse

from pyzbar.pyzbar import decode
import cv2


class Camera1:
    """Camera1 class"""

    def __init__(self):
        """
        The constructor for Camera1 class.
        It Subscribe a ROSTOPIC /eyrc/vb/camera_1/image_raw.
        """
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw",
                                          Image,
                                          self.callback)
        self._colours_list = []
        self._packages_list = []

    def handel_send_colours_packages(self, req):
        """
        Publish list of colors and packages respectively
        on ROSSERVICE eyrc/vb/sendcolourspackages.

        Parameters:
            req (bool): Request for ROSSERVICE.

        Returns:
            jaishreeram: Response of ROSSERVICE.
        """
        jaishreeram = sendColoursPackagesResponse()
        if req:
            jaishreeram.colours = self._colours_list
            jaishreeram.packages = self._packages_list
        return jaishreeram

    def get_qr_data(self, arg_image):
        """
        Decode qr data and check which package is what color.
        """
        qr_result = decode(arg_image)

        if len(qr_result) > 0:
            colours = []
            packages = []

            # Check all qr data in list qr_result
            for i in qr_result:
                colours.append(i.data)

                # Check in which shelf's row package is present
                if (i.rect.top >= 310 and i.rect.top <= 320):
                    # Check in which shelf's column package is present
                    if (i.rect.left >= 120 and i.rect.left <= 135):
                        box = "packagen00"
                    elif (i.rect.left >= 310 and i.rect.left <= 320):
                        box = 'packagen01'
                    else:
                        box = 'packagen02'
                elif (i.rect.top >= 490 and i.rect.top <= 500):
                    if (i.rect.left >= 120 and i.rect.left <= 135):
                        box = "packagen10"
                    elif (i.rect.left >= 310 and i.rect.left <= 320):
                        box = 'packagen11'
                    else:
                        box = 'packagen12'
                elif (i.rect.top >= 640 and i.rect.top <= 650):
                    if (i.rect.left >= 120 and i.rect.left <= 135):
                        box = "packagen20"
                    elif (i.rect.left >= 310 and i.rect.left <= 320):
                        box = 'packagen21'
                    else:
                        box = 'packagen22'
                else:
                    if (i.rect.left >= 120 and i.rect.left <= 135):
                        box = "packagen30"
                    elif (i.rect.left >= 310 and i.rect.left <= 320):
                        box = 'packagen31'
                    else:
                        box = 'packagen32'

                # Update packages list
                packages.append(box)
                start_point = (i.polygon[0].x, i.polygon[0].y)
                end_point = (i.polygon[2].x, i.polygon[2].y)
                colour = (0, 0, 0)
                thikness = -1
                cv2.rectangle(arg_image,
                              start_point,
                              end_point,
                              colour,
                              thikness)
            self._colours_list = colours
            self._packages_list = packages
            # cv2.imshow('ractangle', arg_image)
        else:
            pass

    def callback(self, data):
        """
        This is a callback function of ROSTOPIC /eyrc/vb/camera_1/image_raw
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        image = cv_image

        # Cleaning image before decoding
        _, threshold = cv2.threshold(image, 70, 255, cv2.THRESH_TRUNC)
        self.get_qr_data(threshold)
        cv2.waitKey(3)


def main(args):
    """Main"""
    rospy.init_node('node_t5_qr_decode', anonymous=True)
    ic = Camera1()

    # Start ROSERVICE eyrc/vb/sendcolourspackages
    rospy.Service('eyrc/vb/sendcolourspackages',
                  sendColoursPackages,
                  ic.handel_send_colours_packages)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
