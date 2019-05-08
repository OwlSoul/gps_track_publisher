#!/usr/bin/env python2

from __future__ import print_function
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import fastkml.gx
from fastkml import kml
from pykml import parser
import datetime
import rospy
import sys
from os import path
import dateutil.parser
import lxml.etree as et


class ROSNode:
    def __init__(self):
        self.track_file = 'sample_tracks/track-01.kml'

        # Name of the node
        self.node_name = 'gps_track_publisher'

        # Topic to publish the gps data
        self.track_pub = None
        self.track_topic_name = '/gps_track'
        self.queue_size = 10

        # KML track number, default is zero
        self.kml_track_no = 0

        # Speed multiplier, default is 1.0
        self.speed_multiplier = 1.0

        # If True, the node will terminate itself when finished.
        self.exit_when_finished = True

    @staticmethod
    def timestamp(data):
        return (data - datetime.datetime(1970,1,1)).total_seconds()

    def publish_kml(self, filename, track_no):
        xml = et.parse(filename)
        root = xml.getroot()
        print(root.nsmap)
        # Getting KML namespace first
        if 'kml' in root.nsmap:
            kmlns = {'kml': root.nsmap['kml']}
        else:
            kmlns = {'kml': root.nsmap[None]}
        print(kmlns)
        data = root.xpath("//kml:Document/kml:name/text()", namespaces=kmlns)
        print(data)

        placemarks = root.xpath("//kml:Document/kml:Placemark", namespaces=kmlns)
        for i, placemark in enumerate(placemarks):
            print("Placemark #" + str(i) + ":")

            print("  Name: ", end="")
            try:
                print(placemark.xpath("kml:name/text()", namespaces=kmlns)[0])
            except:
                print("")

            print("  Description: ", end="")
            try:
                print(placemark.xpath("kml:description/text()", namespaces=kmlns)[0])
            except:
                print("")

            begin = None
            end = None

            try:
                begin = placemark.xpath("kml:TimeSpan/kml:begin/text()", namespaces=kmlns)[0]
                begin_timestamp = self.timestamp(dateutil.parser.parse(str(begin)).replace(tzinfo=None))
                print("  Begin: ", end="")
                print(str(begin) + " / " + str(begin_timestamp))
            except:
                print(" - ")

            try:
                end = placemark.xpath("kml:TimeSpan/kml:end/text()", namespaces=kmlns)[0]
                end_timestamp = self.timestamp(dateutil.parser.parse(str(end)).replace(tzinfo=None))
                print("  End  : ", end="")
                print(str(end) + " / " + str(end_timestamp))
            except:
                print(" - ")

    def run(self):
        rospy.init_node(self.node_name)

        self.track_pub = rospy.Publisher(self.track_topic_name, NavSatFix, queue_size=self.queue_size)

        self.publish_kml(self.track_file, self.kml_track_no)




if __name__=='__main__':
    node = ROSNode()
    node.run()
    sys.exit(0)