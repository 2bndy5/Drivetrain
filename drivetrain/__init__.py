import adafruit_platformdetect as pfdetect

ON_WINDOWS = pfdetect.platform.platform(
    aliased=1, terse=1).startswith('Windows')

DETECT = pfdetect.Detector()  # object used to detect various information
MODEL = DETECT.board.id  # returns `None` on Windows

ON_RASPI = False if MODEL is None else MODEL.startswith('RASPBERRY_PI')
ON_JETSON = False if MODEL is None else MODEL.startswith('JETSON')
