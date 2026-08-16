Script to create lines to add to udev rules files.

## Installation
Dependencies:
* pyudev

If not already done, create a python virtual environment with:
> python3 -m venv .venv

Activate the virtual environment:
> source .venv/bin/activate

Install requirements:
> pip3 install -r requirements

## Usage
Steps:
* Start the script
> ./udevcreator.py
* Plug in the device, a line should be printed to the console
* Copy the line to the relevant udev rules file, e.g.
> /etc/udev/rules.d/60-gpsd.rules
* Reload udev rules:
> sudo udevadm control --reload-rules
* Trigger the rule manually:
> sudo udevadm trigger
