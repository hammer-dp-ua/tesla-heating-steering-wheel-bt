# Tesla M3 2018 heating steering wheel using ESP32 S3

Original M3 2018 steering wheel was replaced by 2021 M3/MY from Tesla restyle.

Controlled by Bluetooth button:
![Bluetooth button](https://github.com/hammer-dp-ua/tesla-heating-steering-wheel-bt/blob/main/scheme/Bluetooth_button.jpeg)

Heating steering wheel power consumption is about 6 Amps / 100W.
And it turns out there is overcurrent protection in Tesla Body Left Controller.
I just cut pin 23 on J6/X035 connector and connected the wire directly to 12V line using 10A fuse.
![Tesla J6/X035](https://github.com/hammer-dp-ua/tesla-heating-steering-wheel-bt/blob/main/scheme/Tesla_J6-X035.svg)
![Tesla J6/X035 photo](https://github.com/hammer-dp-ua/tesla-heating-steering-wheel-bt/blob/main/scheme/Tesla-J6-X035_photo.jpg)

esp-idf: v5.1.2 stable