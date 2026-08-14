
# Setup

## System packages
System packages needed:
* gpsd
* gpsd-clients

Install with apt:
> sudo apt install gpsd gpsd-clients

## Configure gpsd
Configure GPSD to use only the ZED-F9P on `/dev/ttyUSB1`. Disabling USB auto
detection prevents the older u-blox receiver from also publishing fixes.

Edit `/etc/default/gpsd`:

```ini
DEVICES="/dev/ttyUSB1"
GPSD_OPTIONS="-n"
USBAUTO="false"
```

Restart GPSD:

```bash
sudo systemctl restart gpsd.socket gpsd.service
```

You should now see activity when running cgps
> cgps

## configure gps reader script
The gps reader script needs the paho-mqtt package, which can be installed as follows:
> pip3 install -r requirements.txt
