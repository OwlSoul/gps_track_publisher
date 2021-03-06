#!/usr/bin/env python2

"""
This node will publish GPS data from a pre-recorded track in kml or gpx format to specified ROS node(s).

Current limitations:
  Known to work with kml from the following sources:
     - GeoTracker android app - provides tracks with gx extensions, simply perfect.
     - Tracks from bikemap.net

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
import lxml.etree as et

class ROSNode:
    def __init__(self):
        self.track_file = 'sample_tracks/track-01.kml'

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
        #TODO: Override track duration, this is very useful for tracks without timestamps.
        # Publisher will linearly approximate track to match it with this duration.
        # The limitation is that the speed will be constant during the whole track.
        # If set to 0 the whole track will be published immediately.
        self.override_duration = 3600

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

            # Parsing the LineString thingy here, this usually comes without timedata
            try:
                linestrings = placemark.xpath("kml:LineString", namespaces=nsmap)
                for linestring_no, linestring in enumerate(linestrings):
                    linestring_result = []
                    print("Linestring " + str(linestring_no) + ":")
                    coords = linestring.xpath("kml:coordinates", namespaces=nsmap)
                    for coord in coords:
                        value = str(coord.xpath("text()", namespaces=nsmap)) \
                            .replace('[', ' ') \
                            .replace(']', ' ') \
                            .replace("'", ' ') \
                            .replace('n', ' ') \
                            .split('\\')
                        for entry in value:
                            try:
                                coord_values = list(map(float, iter(entry.split(','))))
                                linestring_result.append({'coord': coord_values,
                                                          'timestamp': 0.0 })
                            except:
                                pass

                        result.append(linestring_result)

            except Exception as e:
                print("LineString: " + str(e))

            # Parsing the Multitrack thingy next
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

                        # Parsing the speed data
                        res_speeds = []
                        try:
                            speed_values = track.xpath("kml:ExtendedData/"
                                                     "kml:SchemaData/"
                                                     "gx:SimpleArrayData[@name='speed']/"
                                                     "gx:value",
                                                     namespaces=nsmap)
                            print("  Loaded " + str(len(speed_values)) + " speed values")
                            for value in speed_values:
                                try:
                                    speed = float(value.xpath("text()", namespaces=nsmap)[0])
                                except Exception as e:
                                    print(str(e))
                                    speed = 0.0
                                finally:
                                    res_speeds.append(speed)

                        except Exception as e:
                            print("gx Speed values: " + str(e))

                        # Parsing the course data
                        res_course = []
                        try:
                            course_values = track.xpath("kml:ExtendedData/"
                                                       "kml:SchemaData/"
                                                       "gx:SimpleArrayData[@name='course']/"
                                                       "gx:value",
                                                       namespaces=nsmap)
                            print("  Loaded " + str(len(course_values)) + " course values")
                            for value in course_values:
                                try:
                                    course = float(value.xpath("text()", namespaces=nsmap)[0])
                                except:
                                    course = None
                                finally:
                                    res_course.append(course)

                        except Exception as e:
                            print("gx Course values: " + str(e))

                        # Filling up the result for this placemark. Usually there should be the same amount of entries here
                        # (coordinates, timestamp etc), but who anyway there are some slight measures to test if it is not.
                        intermediate_result = []

                        for coord in res_coords:
                            intermediate_result.append({'coord': coord})

                        for ts_seq, ts in enumerate(res_timestamps):
                            if ts_seq < len(res_coords):
                                intermediate_result[ts_seq]['timestamp'] = ts

                        for speed_seq, speed in enumerate(res_speeds):
                            if speed_seq < len(res_coords):
                                intermediate_result[speed_seq]['speed'] = speed

                        for course_seq, course in enumerate(res_course):
                            if course_seq < len(res_course):
                                intermediate_result[course_seq]['course'] = course

                        placemark_result.extend(intermediate_result)

            except Exception as e:
                print("gx:MultiTrack: " + str(e))

            # This also should be sorted already, but ANYWAY.
            placemark_result = sorted(placemark_result, key=lambda x: x['timestamp'])

            if placemark_result:
                result.append(placemark_result)

            return result



    def run(self):
        rospy.init_node(self.node_name)

        self.track_pub = rospy.Publisher(self.track_topic_name, NavSatFix, queue_size=self.queue_size)

        data = self.parse_kml(self.track_file)

        print()
        print("The result:")
        for i, entry in enumerate(data):
            print("Entry " + str(i))
            for x in entry:
                pass
                print(" ", x)

if __name__=='__main__':
    node = ROSNode()
    node.run()
    sys.exit(0)