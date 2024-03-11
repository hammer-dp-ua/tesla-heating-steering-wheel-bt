# Tesla M3 2018 heating steering wheel using ESP32 S3

esp-idf: v5.1.2 stable

Original M3 2018 steering wheel was replaced by 2021 M3/MY from Tesla restyle.

Controlled by Bluetooth button:

![Bluetooth button](https://github.com/hammer-dp-ua/tesla-heating-steering-wheel-bt/blob/main/scheme/Bluetooth_button.jpeg)

## Buttons functionality
"+" button increases temperature.

"-" button decreases temperature.

Supported temperatures: 20-30-40°C (1 beep / 2 beeps / 3 beeps).

"Start/Stop" button turns on heating (1 beep) / turns off heating (2 beeps).

"Forward" long press button starts OTA process.

## Tesla power limits
Heating steering wheel power consumption is about 6 Amps / 100W.
And it turns out there is overcurrent protection in Tesla Body Left Controller that turns off heating steering wheel.
I just cut pin 23 (violet, 0.75) on J6/X035 connector and connected the wire directly to 12V line using 10A fuse.

![Tesla J6/X035](https://github.com/hammer-dp-ua/tesla-heating-steering-wheel-bt/blob/main/scheme/Tesla_J6-X035.svg)
![Tesla J6/X035 photo](https://github.com/hammer-dp-ua/tesla-heating-steering-wheel-bt/blob/main/scheme/Tesla-J6-X035_photo.jpg)