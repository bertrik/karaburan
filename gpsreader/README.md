
# Setup

## System packages
System packages needed:
* gpsd
* gpsd-clients

Install with apt:
> apt install gpsd gpsd-clients

## Serial port rules
To set up the serial port:
* insert the GPS into USB
* verify USB vendor and product id
>  lsusb
* create an udev rule
  sudo nano /etc/udev/rules.d/99-usb-serial.rules
* add a line
> SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyGPS", GROUP="dialout"
* reload the udev rules
> sudo udevadm --control reload
* trigger the udev rules
> sudo udevadm trigger
(or just unplug/replug the USB device)

You should now have a /dev/ttyGPS device

## Configure gpsd
To configure gpsd:
* open its configuration file
  sudo nano /etc/default/gpsd
* add/edit the following lines
  DEVICES="/dev/ttyGPS"
  GPSD_OPTIONS="-Gn ntrip://user:pass@ntrip.kadaster.nl:2101/CBW100NLD0"
  USBAUTO="false"
* restart gpsd
  sudo systemctl restart gpsd gpsd.socket

You should now see activity when running cgps
> cgps

## configure gps reader script
The gps reader script needs the paho-mqtt package, which can be installed as follows:
> pip3 install -r requirements.txt

