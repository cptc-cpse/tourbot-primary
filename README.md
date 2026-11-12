The current primary code repository for the tourbot project. 

Installation Instructions: 
Windows
```
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Linux
```
apt install python3.13-venv
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```
Then check usb

## LiDAR serial port

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

If opening the device gives a permission error, add your user to the serial
device's group (commonly `dialout` on Linux), then sign out and back in.