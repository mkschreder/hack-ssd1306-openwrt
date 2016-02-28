SSD1306 OLED Display On OpenWRT and Hootoo Box
----------------------------------------------

As a fun thing you can do to learn about linux device tree and playing with
linux device drivers is connecting an i2c display through a usb dongle to a
linux box and then making it work with a framebuffer driver. 

This for example allows you to write pixels to the display by using a standard
framebuffer file (/dev/fbX) which a linux application can then easily access. 

To do this you need to get two drivers from this repo. First is a ch341 driver
package for openwrt which I have confirmed works and another one is the ssd1306
framebuffer driver which I modified to work on OpenWRT. 

![Image](/screenshot.jpg)

Some points of interest: 

- The ssd1306 driver interfaces with standard linux framebuffer system. I have
  modified the ssd1307fb driver already in the main linux tree because I could
  not make it work without having to modify it. After replacing init sequence
  to use one used by the u8g library things work a lot better. 
- To get the display driver to recognise the display, I register it from the
  command line like this (0x3c is the i2c address of my display): 
	
	echo "ssd1306fb 0x3c" > /sys/bus/i2c/devices/i2c-0/new_device

- The driver stores written data in a buffer and schedule work for putting it
  to the display. I removed the blocking updates from the original driver so
  that update is now triggered to happen in the background. If you are writing
  to the display while an update is in progress, the data will be partially new
  on the screen, but a new full update will happen after current one is done.
- The i2c interface is very slow. Current implementation has to split writes
  into chunks of max 25 bytes due to limitations of the i2c converter. More
  info needed. There is probably lots of room for improvement here. But that's
  beyond this little test.  

COPYING 
-------

- Copyright 2016 Martin Schröder <mkschreder.uk@gmail.com>
- Copyright [ original authors of driver code ]

You may use this code according to their original licenses. 
