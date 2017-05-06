#!/usr/bin/env python
# Converts raw RGB565 video to MP4/AVI

from sys import argv, exit
from array import array
from subprocess import call

buf=None
TMP_FILE = "/tmp/video.raw"

if (len(argv) != 4):
    print("Usage: encode_raw input.raw output.avi fps")
    exit(1)

with open(argv[1], "rb") as f:
    buf = array("H", f.read())

#Swap not needed if rgb565be is supported
buf.byteswap()
with open(TMP_FILE, "wb") as f:
    f.write(buf.tostring())

cmd = "ffmpeg -vcodec rawvideo -r %d -f rawvideo -pix_fmt rgb565 -s 160x120 -i %s -vcodec mpeg4 %s"%(int(argv[3]), TMP_FILE, argv[2])
call(cmd.split())
