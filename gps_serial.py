# The MIT License (MIT)
#
# Copyright (c) 2017 Damien P. George
# Copyright (c) 2019 Brendan Doherty
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
========================================
gps_serial
========================================

Yet another NMEA sentence parser for serial UART based GPS modules. This implements the threaded module for [psuedo] asynchronous applications. CAUTION: The individual satelite info is being ignored until we decide to support capturing it from the GPS module's output.
"""
__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/DVC-Viking-Robotics/GPS_Serial.git"
import time
import threading
import serial

# default location hard-coded to DVC Engineering buildings' courtyard
DEFAULT_LOC = {'lat': 37.96713657090229, 'lng': -122.0712176165581}

def _convert2deg(nmea):
    """VERY IMPORTANT needed to go from format 'ddmm.mmmm' into decimal degrees"""
    if nmea is None or len(nmea) < 3:
        return None
    nmea = float(nmea)
    return (nmea // 100) + (nmea - ((nmea // 100) * 100)) / 60

class GPSserial:
    """
    :param int address: The serial port address that the GPS module is connected to. For example, on the raspberry pi's GPIO pins, this is ``/dev/ttyS0``; on windows, this is something like ``com#`` where # is designated by windows.
    :param int timeout: Specific number of seconds till the threaded :class:`~serial.Serial`'s ``readline()`` operation expires. Defaults to 1 second.
    """
    def __init__(self, address, timeout=1.0):
        self._dummy = False
        try:
            self._ser = serial.Serial(address, timeout=timeout)
            self._line = self._ser.readline()  # discard any garbage artifacts
            self._gps_thread = None
            print('Successfully opened port', address, 'to GPS module')
        except serial.SerialException:
            self._dummy = True
            print('unable to open serial GPS module @ port', address)
        self._lat = DEFAULT_LOC['lat']
        self._lng = DEFAULT_LOC['lng']
        self._utc = None
        self._line = ""
        self._speed = {"knots": 0.0, "kmph": 0.0}
        self._course = {"true": 0.0, "mag": 0.0}
        self._sat = {"connected": 0, "view": 0, "quality": "Fix Unavailable"}
        self._altitude = 0.0
        # self.azimuth = 0.0
        # self.elevation = 0.0
        self._data_status = 'Data not valid'
        self._fix = "no Fix"
        self._rx_status = "unknown"
        self._pdop = 0.0
        self._hdop = 0.0
        self._vdop = 0.0

    @property
    def lat(self):
        """This attribute holds the latitude coordinate that was most recently parsed from the GPS module's data output."""
        return self._lat
    @property
    def lng(self):
        """This attribute holds the longitude coordinate that was most recently parsed from the GPS module's data output."""
        return self._lng
    @property
    def utc(self):
        """This attribute holds a tuple of time & date data that was most recently parsed from the GPS module's data output. This tuple conforms with python's time module functions."""
        return self._utc
    @property
    def speed_knots(self):
        """This attribute holds the speed (in nautical knots) that was most recently parsed from the GPS module's data output."""
        return self._speed['knots']
    @property
    def speed_kmph(self):
        """This attribute holds the speed (in kilometers per hour) that was most recently parsed from the GPS module's data output."""
        return self._speed['kmph']
    @property
    def sat_connected(self):
        """This attribute holds the number of connected GPS satelites that was most recently parsed from the GPS module's data output."""
        return self._sat['connected']
    @property
    def sat_view(self):
        """This attribute holds the number of GPS satelites in the module's view that was most recently parsed from the GPS module's data output."""
        return self._sat['view']
    @property
    def sat_quality(self):
        """This attribute holds the description of the GPS satelites' quality that was most recently parsed from the GPS module's data output."""
        return self._sat['quality']
    @property
    def course_true(self):
        """This attribute holds the course direction (in terms of "true north") that was most recently parsed from the GPS module's data output."""
        return self._course['true']
    @property
    def course_mag(self):
        """This attribute holds the course direction (in terms of "magnetic north") that was most recently parsed from the GPS module's data output."""
        return self._course['mag']
    @property
    def altitude(self):
        """This attribute holds the GPS antenna's altitude that was most recently parsed from the GPS module's data output."""
        return self._altitude
    @property
    def fix(self):
        """This attribute holds the description of GPS module's fix quality that was most recently parsed from the GPS module's data output."""
        return self._fix
    @property
    def data_status(self):
        """This attribute holds the GPS module's data authenticity that was most recently parsed from the GPS module's data output."""
        return self._data_status
    @property
    def rx_status(self):
        """This attribute holds the GPS module's receiving status that was most recently parsed from the GPS module's data output."""
        return self._rx_status
    @property
    def pdop(self):
        """This attribute holds the GPS module's positional dilution of percision that was most recently parsed from the GPS module's data output."""
        return self._pdop
    @property
    def vdop(self):
        """This attribute holds the GPS module's vertical dilution of percision that was most recently parsed from the GPS module's data output."""
        return self._vdop
    @property
    def hdop(self):
        """This attribute holds the GPS module's horizontal dilution of percision that was most recently parsed from the GPS module's data output."""
        return self._hdop

    def _parse_line(self, string):
        found = False
        if string.find('GLL') != -1:
            found = True
            arr = string.rsplit(',')[1:]
            # it would probably be helpful to other location-based APIs to have the
            # corrdinates also saved in the original 'DDMM.SS [cardinal direction]'
            self._lat = _convert2deg(arr[0])
            if arr[1] != 'N' and arr[1] is not None:
                self._lat *= -1
            self._lng = _convert2deg(arr[2])
            if arr[3] != 'E' and arr[3] is not None:
                self._lng *= -1.0
            type_state = {'A': 'data valid', 'V': 'Data not valid'}
            self._data_status = type_state[arr[5]]
        elif string.find('VTG') != -1:
            arr = string.rsplit(',')[1:]
            if len(arr[0]) > 1:
                self._course["true"] = float(arr[0])
            if len(arr[1]) > 1:
                self._course["mag"] = float(arr[1])
            if len(arr[2]) > 1:
                self._speed["knots"] = float(arr[2])
            if len(arr[3]) > 1:
                self._speed["kmph"] = float(arr[3])
        elif string.find('GGA') != -1:
            type_state = [
                "Fix Unavailable", "Valid Fix (SPS)", "Valid Fix (GPS)", "unknown1", "unknown2", "unknown3"]
            arr = string.rsplit(',')[1:]
            self._sat["quality"] = type_state[int(arr[5])]
            self._sat["view"] = int(arr[6])
            if len(arr[8]) > 1:
                self._altitude = float(arr[8])
        elif string.find('GSA') != -1:
            arr = string.rsplit(',')[1:]
            type_fix = ["No Fix", "2D", "3D"]
            self._fix = type_fix[int(arr[1]) - 1]
            self._pdop = float(arr[14])
            self._hdop = float(arr[15])
            self._vdop = float(arr[16][:-3])
        elif string.find('RMC') != -1:
            status = {"V": "Warning", "A": "Valid"}
            arr = string.rsplit(',')[1:]
            self._rx_status = status[arr[1]]
            if len(arr[0]) > 1 and len(arr[8]) > 1:
                self._utc = time.struct_time((2000+int(arr[8][4:6]), int(arr[8][2:4]), int(arr[8][0:2]), int(arr[0][0:2]), int(arr[0][2:4]), int(arr[0][4:6]), 0, 0, -1))
        elif string.find('GSV') != -1:
            arr = string.rsplit(',')[1:]
            self._sat['connected'] = arr[0]
            # ignoring data specific to individual satelites
            # self.elevation = int(arr[4])
            # self.azimuth = int(arr[5])
            # print('sat["view"]:', self.sat["connected"], 'elevation:', self.elevation, 'Azimuth:', self.azimuth)
        return found

    def _threaded_read(self, raw):
        found = False
        while not found:
            self._line = self._ser.readline()
            try:
                self._line = str(self._line, 'ascii').strip()
            except UnicodeError:
                continue # there was undecernable garbage data that couldn't get encoded to ASCII
            if raw:
                print(self._line)
            # found = true if gps coordinates are captured
            found = self._parse_line(self._line)

    def get_data(self, raw=False):
        """
        This function only starts the process of parsing the data from a GPS module (if any).

        :param bool raw: `True` prints the raw data being parsed from the GPS module. `False` doesn't print the raw data. Defaults to `False`.

        :returns: the last latitude and longitude coordinates obtained from either object instantiation (zero values) or previously completed parsing of GPS data.
        """
        if not self._dummy:
            if self._gps_thread is not None:
                self._gps_thread.join()
            self._gps_thread = threading.Thread(
                target=self._threaded_read, args=[raw])
            self._gps_thread.start()
        return {"lat": self.lat, "lng": self.lng}
