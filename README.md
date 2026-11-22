The current primary code repository for the tourbot project. 

# Running it:
On the first time, go through the relevant deployment instructions, make sure you have Mosquitto running then run the following in the repository's directory.

## Windows
``` 
.\.venv\Scripts\Activate.ps1
python main.py
```

## Linux
```
source .venv/bin/activate
python3 main.py
```

# Deployment Instructions: 
## Windows
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Linux
```
apt install python3.13-venv
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

### OAK on linux

Open /etc/udev/rules.d/80-movidius.rules and add
`SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"`
then run 
`sudo udevadm control --reload-rules && sudo udevadm trigger`


### LiDAR serial port

The LiDAR script uses `COM3` by default on Windows and `/dev/ttyUSB0` on Linux.
Linux device names can vary; check which one was assigned with:

```sh
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

If the LiDAR is listed under a different name, set `LIDAR_PORT` to that device
when starting the application, for example:

```sh
LIDAR_PORT=/dev/ttyACM0 python3 main.py
```

If opening the device gives a permission error, check its group (the group is
typically the fourth column in the output):

```sh
ls -l /dev/ttyUSB0
```

Add your user to that group (commonly `dialout` on Linux):

```sh
sudo usermod -aG dialout "$USER"
```

Replace `dialout` if the device belongs to a different group. Sign out of your
Linux session and back in for the new group membership to take effect. You can
verify it with `groups`.