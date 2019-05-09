#!/usr/bin/env python2

"""
Print important data about GPS track on screen.
"""


__author__ = "Yury D."
__credits__ = ["Yury D."]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Yury D."
__email__ = "TheOwlSoul@gmail.com"
__status__ = "Alpha"

import argparse
import sys
import os
from gps_common_lib import GPSFunctions

class Application:
    def __init__(self):
        self.filename = ''

    def parse_arguments(self):
        parser = argparse.ArgumentParser(description="Print the info about supproted tracks inside GPS track file.\n"
                                                     "This will help you to decide which track to use for publisher "
                                                     "node.",
                                         formatter_class=argparse.RawTextHelpFormatter)
        parser.add_argument("-v", "--version", action="store_true", default=False,
                            help="show version info")
        parser.add_argument("filename", default="", nargs='?',
                            help="name of the file containing GPS track (kml or gpx)")

        args = parser.parse_args()
        if args.version:
            print(__version__)
            sys.exit(0)

        if args.filename:
            if os.path.isfile(args.filename):
                self.filename = args.filename
            else:
                print("Specified file does not exist")
                sys.exit(1)
        else:
            print("No filename provided!")
            sys.exit(2)


    def run(self):
        self.parse_arguments()
        gps_tracks = GPSFunctions.parse_kml(self.filename)
        for placemark_no, placemark in enumerate(gps_tracks):
            print("Placemark " + str(placemark_no))
            for entry in placemark:
                pass
                #print(entry)



if __name__=='__main__':
    app = Application()
    app.run()
    sys.exit(0)