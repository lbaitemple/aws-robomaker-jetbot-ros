#!/bin/sh

a=`i2cdetect -y -r 1 | grep 5d | wc -l`
if [ $a -gt 0 ]
then
   sudo chmod 0666 /dev/i2c-1
   cd /greengrass/ggc/core/
   sudo ./greengrassd start
else
   echo "NO motor driver is detected"
fi
