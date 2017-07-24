# fm_transmitter

Project forked from [PNPtutorials](https://github.com/PNPtutorials/FM_Transmitter_RPi3).

Use Raspberry Pi as FM transmitter. Works on any RPi board.

This project uses the general clock output to produce frequency modulated radio
communication. It is based on idea originaly posted [here][orig] but does not 
use DMA controller in order to distribute samples to output (clock generator),
so sound quality is worse as in PiFm project and only mono transmition is available 
but this makes possible to run it on all kind of boards.

[orig]: http://icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter

## How to use it

To compile this project use commands below:
```
sudo apt-get install make gcc g++
make packages
make
``` 

Additional remakes can be triggered using just `make` or `make clean && make`.

Then you can use it by typing:
```
sudo ./fm_transmitter [-f frequencyMHz=100.0] [-s spreadMHz=0.078] [-r {loop if set}] [-v {verbose logging if set}] [-d alsa-device] filename
```

For example, to read the `star_wars.wav` WAV file, use

```
sudo ./fm_transmitter -f 100.1 -s 0.078 star_wars.wav
```


### ALSA sound-card

We can also read from an ALSA sound card. For example, a USB sound card will usually
just work if you type

```
sudo ./fm_transmitter -f 100.6 -
```

If you need to change the device number (e.g., you have multiple sound cards),
then you can change the device:

```
sudo ./fm_transmitter -f 100.1 -d plughw:1,0 -
```

The available device numbers can be found using `arecord -l`. 

  > Note, a slightly faster sound card may be available using `hw:1,0` instead 
  > of `plughw:1,0`. The difference is that `hw` does not do any software 
  > resampling, so it may fail if the sampling rate (44.1kHz) is not hardware 
  > supported.

## Daemonization

If you want this to be run on startup, copy the file "./system_configuration/fm_transmitter.service" to /lib/systemd/system/ 
then, on the pi:
```
sudo systemctl enable fm_transmitter.service
sudo systemctl start fm_transmitter.service

```
## Law
Please keep in mind that transmitting on certain frequencies without special 
permissions may be illegal in your country.

