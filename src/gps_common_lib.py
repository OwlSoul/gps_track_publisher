"""
Common library for various GPS functions of 'gps_track_publisher' node.
"""

from __future__ import print_function
import datetime
import dateutil.parser
import lxml.etree as et


class GPSFunctions:
    def __init__(self):
        pass

    @staticmethod
    def timestamp(data):
        """
        Gets a timestamp from the ISO date format. Python2 does not have this "from the box"
        :param data: timestamp in ISO format
        :return: Unix timestamp
        """
        return (data - datetime.datetime(1970, 1, 1)).total_seconds()

    @staticmethod
    def parse_line_string(placemark, nsmap={}):
        """
        Parsing the LineString element, this usually comes without time data whatsoever
        :param placemark: Placemark element
        :param nsmap: namespace map
        :return: array of coordinates (lat, lon, alt): [[float, float, float], [float, float, float], ...]
        """
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
                    print("  Contains " + str(len(value)) + " coordinates")
                    for entry in value:
                        try:
                            coord_values = list(map(float, iter(entry.split(','))))
                            linestring_result.append({'coord': coord_values,
                                                      'timestamp': 0.0})
                        except:
                            pass

                    return linestring_result

        except Exception as e:
            print("LineString: " + str(e))
            return []

    @staticmethod
    def parse_kml(filename):
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
            print("Setting default NSMAP to 'kml'")
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

            print("  Begin: ", end="")
            try:
                begin = placemark.xpath("kml:TimeSpan/kml:begin/text()", namespaces=nsmap)[0]
                begin_timestamp = GPSFunctions.timestamp(dateutil.parser.parse(str(begin)).replace(tzinfo=None))
                print(str(begin) + " / " + str(begin_timestamp))
            except:
                print(" - ")

            print("  End  : ", end="")
            try:
                end = placemark.xpath("kml:TimeSpan/kml:end/text()", namespaces=nsmap)[0]
                end_timestamp = GPSFunctions.timestamp(dateutil.parser.parse(str(end)).replace(tzinfo=None))
                print(str(end) + " / " + str(end_timestamp))
            except:
                print(" - ")
            print("")

            # Parsing the LineString thingy here, this usually comes without timedata
            linestring_result = GPSFunctions.parse_line_string(placemark, nsmap)
            result.append(linestring_result)

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
                                when_ts = GPSFunctions.timestamp(dateutil.parser.parse(str(whentime)).replace(tzinfo=None))
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
            try:
                placemark_result = sorted(placemark_result, key=lambda x: x['timestamp'])
            except Exception as e:
                print("Failed to sort gx:MultiTrack based on timestamps, are they even there?")

            if placemark_result:
                result.append(placemark_result)

            return result

