# Notes

Freeform notes as things are getting set up.


## Dev Environment

These might be useful:

### pigpio

Install `pigpio` following the directions [here](http://abyz.co.uk/rpi/pigpio/download.html):

```bash
rm pigpio.zip
sudo rm -rf PIGPIO
wget abyz.co.uk/rpi/pigpio/pigpio.zip
unzip pigpio.zip
cd PIGPIO
make
sudo make install
```


### piscope

Install `piscope` following the directions [here](http://abyz.co.uk/rpi/pigpio/piscope.html):

```bash
wget abyz.co.uk/rpi/pigpio/piscope.tar
tar xvf piscope.tar
cd PISCOPE
make hf
make install
```


### Emacs

A few tricks:

   1. Install the tutorial shortcuts following the instructions here
   [here](https://tuhdo.github.io/c-ide.html#outline-container-orgheadline0a).
   2. If you are using a mac, just make sure that you fix your meta key:
      a. Open a `Terminal`
      b. Select preferences, then go to `Keyboard` and click 
      `Use Option key as Meta`.
   3. Install markdown mode (assuming you have installed the tutorial as above)
      using `M-x package-install RET markdown-mode RET`.

See the documentation [here](https://tuhdo.github.io/c-ide.html#sec-2) for how to use 
this stuff.

#### Company mode

I have a fetish for code completion, and to get that in C++, I used
`company-mode`.  This required a few steps:

 First, install `cmake` and some other goodies on the RPi

```
sudo apt-get install cmake libclang-3.5-dev
sudo apt-get install mlocate
sudo apt-get updatedb
```

Now open `emacs` and do the following:

  1. [Install irony](https://github.com/Sarcasm/irony-mode): `M-x package-install RET irony RET`
  1. Build and install the `irony-server`*: `M-x irony-install-server RET`. **
  1. Add the following line to the `.clang_complete` file: `-Imy_include`.
  
**If the server doesn't build automatically (it _should_ if you installed the goodies above), 
you can try the following:
```
mkdir /tmp/irony-build
cd /tmp/irony-build
cmake -DLIBCLANG_LIBRARY=/usr/lib/llvm-3.5/lib/libclang.so -DLIBCLANG_INCLUDE_DIR=/usr/lib/llvm-3.5/include/ ~/.emacs.d/ /home/pi/.emacs.d/irony/ /home/pi/.emacs.d/elpa/irony-20170427.1601/server/
make
```
You may need to replace the paths above with system paths. In particular, you need
to locate the following:
```
locate libclang.so
locate clang-c/Index.h
```
See [here](https://github.com/Sarcasm/irony-mode/issues/331) for a related Github issue.


### Install GNU global and `gtags`

First, run the following script to install GNU Global

```
#!/bin/bash
# instGlobal.sh
 
echo "instGlobal.sh ...."
  
echo "install package for GNU global..."
sudo apt-get update
sudo apt-get -y install curl
sudo apt-get -y install wget
sudo apt-get -y install ncurses-dev
sudo apt-get -y install exuberant-ctags
sudo apt-get -y install python
  
CUR=`pwd`
echo "install GNU global..."
wget http://tamacom.com/global/global-6.4.tar.gz
tar zxvf global-6.4.tar.gz
cd global-6.4
./configure --with-exuberant-ctags=/usr/bin/ctags
make
sudo make install
cd $CUR

     
echo "install pip..."
curl -kL https://raw.github.com/pypa/pip/master/contrib/get-pip.py | sudo python
       
sudo pip install pygments==1.6

echo "$0 done."
```

Next, install `ggtags` using `MELPA`:

`M-x package-install RET ggtags RET`


### Semantic-refactor

Sometimes it's really nice to do rapid code refactoring. Here, Semantic Refactoring
comes in handy. To install the package:

   1. `M-x list-packages`, then choose `srefactor` and press `i` and `x` to install.
   1. 
   

## Using `mosh`

Since we may drop a connection, it's better and more convenient to use the 
mobile shell `mosh` than ssh. 

  1. Easiest way on a mac: `brew install mosh`.
  1. On the RPi: `sudo apt-get install mosh`.
  1. [Genreal instructions](https://mosh.org/#getting)
   

