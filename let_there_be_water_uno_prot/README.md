
If you want to use Adafruit’s excellent USBtinyISP programmer with Ubuntu Linux, you’ll probably run into this error:

avrdude: Warning: cannot open USB device: Permission denied

Here’s a quick dump of what should help:

In a terminal, type:
sudo nano /etc/udev/rules.d/99-USBtiny
Then paste
SUBSYSTEM=="usb", ATTR{idVendor}=="1781", ATTR{idProduct}=="0c9f", GROUP="adm", MODE="0666"
into the new file. Write it out by pressing Ctrl-O.

sudo usermod -a -G plugdev YOURUSERNAME
sudo service udev restart
sudo udevadm control --reload-rules

Insert the USBtinyISP again.