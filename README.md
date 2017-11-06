# dooper

Scripts for use with ESP8266 for controlling various home automation peripherals. Customized for my purposes but feel free to edit to your liking.

Works well with the [WEMOS D1 mini v2.3.0](https://wiki.wemos.cc/products:d1:d1_mini) ESP8266 board.

Provides a simplistic web interface on port 80, and also sends UDP packets to a [Homebridge](https://github.com/nfarina/homebridge)
server running:

* [homebridge-udp-json](https://github.com/rxseger/homebridge-udp-json)

## Wiring

* V3, GND
* D2 to BME280 SDA (I2C data)
* D1 to BME280 SCL (I2C clock

## See also
* [cooper](https://github.com/rxseger/cooper/)

## License

MIT
