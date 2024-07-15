
# Setup

## System packages
System packages needed:
* gpsd
* gpsd-clients

Install with apt:
> sudo apt install gpsd gpsd-clients

## Serial port rules
To set up the serial port:
* insert the GPS into USB
* verify USB vendor and product id
>  lsusb
* create a custom udev gpsd.rules file
> sudo nano /etc/udev/rules.d/60-gpsd.rules
* add a line with the USB vendor and USB product id
> ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="gps%n", TAG+="systemd", ENV{SYSTEMD_WANTS}="gpsdctl@%k.service"
* reload the udev rules
> sudo udevadm --control reload
* replug the GPS USB serial device and watch the logs
> sudo journalctl -f

You should now have a /dev/gpsX device.

## Configure gpsd
To configure gpsd:
* open its configuration file
>  sudo nano /etc/default/gpsd
* add/edit the following lines
>  DEVICES=""
>  GPSD_OPTIONS="-Gn -s 38400 ntrip://user:pass@ntrip.kadaster.nl:2101/CBW100NLD0"
>  USBAUTO="true"
* restart gpsd
>  sudo systemctl restart gpsd gpsd.socket

You should now see activity when running cgps
> cgps

## configure gps reader script
The gps reader script needs the paho-mqtt package, which can be installed as follows:
> pip3 install -r requirements.txt

