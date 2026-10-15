#!/usr/bin/env python3
# Limit only this display process; publisher data and system BLAS are unchanged.
import ctypes
import ctypes.util
import sys
library = ctypes.util.find_library('opencv_core')
if not library:
    raise RuntimeError('System OpenCV core library not found')
opencv = ctypes.CDLL(library)
opencv.cvSetNumThreads(1)
assert opencv.cvGetNumThreads() == 1
from rqt_gui.main import Main
sys.exit(Main().main(sys.argv))
