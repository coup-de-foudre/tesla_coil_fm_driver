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

In order to install the system daemon managed by `systemd`, use 
`make daemon`. You can uninstall the daemon using `make uninstall`. 


### Useful Systemd Commands:


#### Job Control

  - ```systemctl start fm_transmitter``` - start process
  - ```systemctl stop fm_transmitter``` - graceful stop process - this usually works
  - ```systemctl kill fm_transmitter``` - kill process
  - ```sudo systemctl restart fm_transmitter``` - restart service manually ( it will restart by itself on process exit)


#### Status

  - ```systemctl``` - list all systemd services, you should see fm_transmitter in here
  - ```systemctl | grep fm_transmitter``` - terser way of seeing if fm_transmitter is running
  - ```systemctl status fm_transmitter``` - See detailed status for fm_transmitter
  - ```sudo journalctl -u fm_transmitter``` - See ouput of fm_transmitter


#### Development`

  - ```sudo systemctl daemon-reload``` - reload config after editing (follow with systemctl restart)


#### Additional daemonization customization

In order want this to run on startup, we copy one of the config files in 
`system_configuration into /lib/systemd/system/fm_transmitter.service` 
(no extension). There are two files in there, one will play starwars, the 
other from an ALSA device.  By default, the makefile copies the one that 
uses the ALSA device.
 
To enable/start manually, run
```
sudo systemctl enable fm_transmitter.service
sudo systemctl start fm_transmitter.service
```

The Pi should start to broadcast, and will restart when the program ends, 
and on boot. 

Should you want to edit the options passed to `fm_transmitter`, you can 
edit the `ExecStart` line in `fm_transmitter.service`.


## Law

Please keep in mind that transmitting on certain frequencies without special 
permissions may be illegal in your country.

