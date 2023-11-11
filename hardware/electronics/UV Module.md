# UV MODULE
This document shows how to connect and program the CJMCU UV Module with an ESP32. 
## About CJMCU UV
The CJMCU UV Module is a serial UV transmitter. It has a simple programming and layout, but a complex sensor. 
The core of it is GUVA-S12SD, whose datasheet is provided here: 
[See datasheet](https://cdn-shop.adafruit.com/datasheets/1918guva.pdf)

## Connecting CJMCU UV to the ESP32 board
It is as simple as connecting the output to an analog pin. 
<div align = center>
<img src="https://github.com/albacorreal/infind/blob/main/multimedia/cjmcu-guva-s12sd.jpg" width = 400 px />
</div>

## Programming the CJMCU UV module with Serial communication.
No sample code is needed. You just have to apply the command: 
~~~C++
uv = analogRead(PIN); 
~~~
