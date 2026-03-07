
## Deploying from desktop

On the rPi web dashboard:

1) Make the rPi writable by selecting the "Writable" tab
2) In the rPi web dashboard Application tab, select the "Uploaded Python file"
   option for Application
3) Click "Browse..." and select the "multiCameraServer.py" file in
   your desktop project directory
4) Click Save

The application will be automatically started.  Console output can be seen by
enabling console output in the Vision Status tab.


## Local vs Robot Setup

For testing, SSH into the raspberrypi:
```bash
ssh pi@wpilibpi.local
```
(PASSWORD: raspberry)

Then run this to access the vision config
```bash
sudo nano /boot/frc.json
```

Now locate "ntmode"
FOR testing locally: set "ntmode" to "server"
FOR roborio: set "ntmode" to "client

*to access network tables on a dashboard, change the target IP to that of the raspberrypi*
