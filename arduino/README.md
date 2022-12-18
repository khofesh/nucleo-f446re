# arduino CLI

https://arduino.github.io/arduino-cli/0.19/

```shell
[fahmad@ryzen ~]$ curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
Installing in /home/fahmad/.local/bin
ARCH=64bit
OS=Linux
Using curl as download tool
Downloading https://downloads.arduino.cc/arduino-cli/arduino-cli_0.29.0_Linux_64bit.tar.gz
arduino-cli  Version: 0.29.0 Commit: 76251df9 Date: 2022-11-17T09:21:40Z installed successfully in /home/fahmad/.local/bin

[fahmad@ryzen ~]$  cd /home/fahmad/GitHub/nucleo-f446re/arduino
[fahmad@ryzen arduino]$  ls
README.md  spi
[fahmad@ryzen arduino]$  cd spi/
[fahmad@ryzen spi]$ arduino-cli board list
Port         Protocol Type              Board Name  FQBN            Core
/dev/ttyACM0 serial   Serial Port (USB) Arduino Uno arduino:avr:uno arduino:avr
/dev/ttyS0   serial   Serial Port       Unknown

[fahmad@ryzen spi]$ arduino-cli core update-index
Downloading index: package_index.tar.bz2 downloaded
[fahmad@ryzen spi]$  ls
spiSlaveRxString.ino
[fahmad@ryzen spi]$ arduino-cli core install arduino:avr
Platform arduino:avr@1.8.6 already installed
[fahmad@ryzen spi]$ arduino-cli core list
ID          Installed Latest Name
arduino:avr 1.8.6     1.8.6  Arduino AVR Boards

[fahmad@ryzen spi]$ arduino-cli compile --fqbn arduino:avr:uno SPISlaveRxString/
Sketch uses 2186 bytes (6%) of program storage space. Maximum is 32256 bytes.
Global variables use 750 bytes (36%) of dynamic memory, leaving 1298 bytes for local variables. Maximum is 2048 bytes.

Used library Version Path
SPI          1.0     /home/fahmad/.arduino15/packages/arduino/hardware/avr/1.8.6/libraries/SPI

Used platform Version Path
arduino:avr   1.8.6   /home/fahmad/.arduino15/packages/arduino/hardware/avr/1.8.6

[fahmad@ryzen spi]$ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno SPISlaveRxString/
```
