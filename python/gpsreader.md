
# Setup

## Installation
System packages needed:
* gpsd
* gpsd-clients

Install the gpsd-clients with apt:
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

You should now see activity when running cgps:
> cgps -um

(-um configures it for metric units)

## Configure gps reader script
The gps reader script needs the paho-mqtt package.
This can be installed into the python virtual environment as follows:
> pip3 install -r requirements.txt

## Interpreting output
The GPS reader script logs the "TPV" report from gpsd.
Relevant fields:
* lat, lon: should be obvious, is in decimal mode
* mode: 0 = no info, 1 = no fix, 2 = 2D-fix, 3 = 3D fix
* status: 0 = no fix, 1 = normal fix, 2 = DGPS fix, 3 = RTK fixed, 4 = RTK float

So normally, we expect mode = 3 (3D fix), and status = 3 (RTK fixed).
