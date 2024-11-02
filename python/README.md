# What
This is a collection of python scripts to control various sensors/actuators
within the Karaburan project.

# How
To use these, you probably want to set up a python virtual environment.
This can be done as follows:

## System-wide one-time setup
You need the following system packages:
* python 3
* python3-pip
* python3-venv
Install them with your package manager, e.g. apt.

## Per clone setup
After getting these files from the github repo, you need to create a python virtual environment.
The virtual environment contains all extra pip packages of the specific version we need for our project.

This can be done as follows:
* create a virtual environment in folder .venv:
```commandline
python3 -m venv .venv
```
* activate the virtual environment
```commandline
source .venv/bin/activate
```
* install the pip packages from the requirements.txt file
```commandline
pip3 install -r requirements.txt
```

## Per runtime session setup
The virtual environment needs to be active to run the scripts:
* activate the virtual environment (if not already active)
```commandline
source .venv/bin/activate
```
The command line now shows a ```(.venv)``` in front of your command prompt
and you can now use the pip packages you installed earlier in the python scripts.

You can switch back out of the virtual environment as follows
```commandline
deactivate
```
