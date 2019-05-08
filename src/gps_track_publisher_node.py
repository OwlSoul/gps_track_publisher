#!/usr/bin/env python2

"""
This node will publish GPS data from a pre-recorded track in kml or gpx format to specified ROS node(s).

Current limitations:
  Known to work with kml from the following sources:
     - GeoTracker android app.

"""

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
from operator import methodcaller
import lxml.etree as et

class ROSNode:
    def __init__(self):
        self.track_file = 'sample_tracks/track-02.kml'

        # Name of the node
        self.node_name = 'gps_track_publisher'

        #TODO: Topic to publish the gps data
        self.track_pub = None
        self.track_topic_name = '/gps_track'
        self.queue_size = 10

        #TODO: Speed multiplier, default is 1.0
        self.speed_multiplier = 1.0

        #TODO: If True, the node will terminate itself when finished.
        self.exit_when_finished = True

        #TODO: Approximate with linear approximation or not. If yes the node will send GPS coordinates from linear
        # approximation between two points if there's a gap between them in the initial data larget than
        # approximate_period
        self.approximate = False
        self.approximate_period = 0.5

    @staticmethod
    def timestamp(data):
        """
        Gets a timestamp from the ISO date format. Python2 does not have this "from the box"
        :param data: timestamp in ISO format
        :return: Unix timestamp
        """
        return (data - datetime.datetime(1970,1,1)).total_seconds()

    def parse_kml(self, filename):
        """
        Parses a KML file
        :param filename: name of the file
        :return: dictionary containing all parsed data
        """

        result = []

        print("KML file: " + filename)
        print()

        xml = et.parse(filename)
        root = xml.getroot()
        nsmap = root.nsmap

        if None in nsmap:
            rospy.loginfo("Setting default NSMAP to 'kml'")
            nsmap['kml'] = nsmap[None]
            del nsmap[None]

        print("Namespaces:")
        for ns, name in nsmap.iteritems():
            print(" ", ns, name)

        print("")

        placemarks = root.xpath("//kml:Document/kml:Placemark", namespaces=nsmap)
        for i, placemark in enumerate(placemarks):
            placemark_result = []

            print("Placemark #" + str(i) + ":")

            print("  Name: ", end="")
            try:
                print(placemark.xpath("kml:name/text()", namespaces=nsmap)[0])
            except:
                print("")

            print("  Description: ", end="")
            try:
                print(placemark.xpath("kml:description/text()", namespaces=nsmap)[0])
            except:
                print("")

            begin = None
            end = None

            try:
                begin = placemark.xpath("kml:TimeSpan/kml:begin/text()", namespaces=nsmap)[0]
                begin_timestamp = self.timestamp(dateutil.parser.parse(str(begin)).replace(tzinfo=None))
                print("  Begin: ", end="")
                print(str(begin) + " / " + str(begin_timestamp))
            except:
                print(" - ")

            try:
                end = placemark.xpath("kml:TimeSpan/kml:end/text()", namespaces=nsmap)[0]
                end_timestamp = self.timestamp(dateutil.parser.parse(str(end)).replace(tzinfo=None))
                print("  End  : ", end="")
                print(str(end) + " / " + str(end_timestamp))
            except:
                print(" - ")
            print("")

            # Parsing the Multitrack thingie next
            try:
                multitracks = placemark.xpath("gx:MultiTrack", namespaces=nsmap)
                for multitrack_no, multitrack in enumerate(multitracks):
                    print("Multitrack " + str(multitrack_no))
                    tracks = multitrack.xpath("gx:Track", namespaces=nsmap)
                    for track_no, track in enumerate(tracks):
                        print("  Track " + str(track_no))
                        # Coordinates
                        res_coords = []

                        try:
                            coords = track.xpath("gx:coord", namespaces=nsmap)
                            for coord in coords:
                                coord_str = coord.xpath("text()")[0]

                                crds = map(float, iter(coord_str.split(" ")))
                                res_coords.append((crds))

                        except:
                            pass
                        finally:
                            print("  Loaded " + str(len(coords)) + " coordinates")

                        # Timestamp
                        res_timestamps = []
                        try:
                            whens = track.xpath("kml:when", namespaces=nsmap)
                            for when in whens:
                                whentime = when.xpath("text()")[0]
                                when_ts = self.timestamp(dateutil.parser.parse(str(whentime)).replace(tzinfo=None))
                                res_timestamps.append(when_ts)
                        except:
                            pass
                        finally:
                            print("  Loaded " + str(len(whens)) + " timestamps")

                        # Filling up the result for this placemark. Usually there should be the same amount of entries here
                        # (coordinates, timestamp etc), but who anyway there are some slight measures to test if it is not.
                        intermediate_result = []

                        for coord in res_coords:
                            intermediate_result.append({'coord': coord,
                                                     'timestamp': 0.0})

                        for ts_seq, ts in enumerate(res_timestamps):
                            if ts_seq < len(res_coords):
                                intermediate_result[ts_seq]['timestamp'] = ts

                        placemark_result.extend(intermediate_result)

            except Exception as e:
                print(str(e))

            # This also should be sorted alreasy, but ANYWAY.
            placemark_result = sorted(placemark_result, key=lambda x: x['timestamp'])

            result.append(placemark_result)

            return result



    def run(self):
        rospy.init_node(self.node_name)

        self.track_pub = rospy.Publisher(self.track_topic_name, NavSatFix, queue_size=self.queue_size)

        data = self.parse_kml(self.track_file)

        for entry in data:
            for x in entry:
                pass
                print(x)

if __name__=='__main__':
    node = ROSNode()
    node.run()
    sys.exit(0)