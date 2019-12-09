# OctoROS
This projects aims to be a bridge between 3D Printers and ROS. 
It uses OctoPrint to control the printer, and get the info that is sent to ROS topics. 


## Tested Environment

- Makerbot Replicator 2X (Firmware 7.6), Ultimaker 2+ and GtMax3D
- Ubuntu LTS 16.04.06 (Xenial Xerus)
- ROS Kinetic   
- Python 2.7
- Octoprint 1.3.11

## Prerequisites

Make sure you have Python 2.7 along with pip installed, if not so:

```$ wget https://bootstrap.pypa.io/get-pip.py```

```$ sudo python2.7 get-pip.py```

Since OctoROS relies on the ```requests``` package, it's better to install it in forehand:

```$ sudo pip2.7 install requests```

Now we can install OctoPrint, the detailed instructions are available at https://github.com/foosel/OctoPrint. But the easiest way that I see and in order to integrate the application with ROS is with the following instructions:

1. Checkout OctoPrint:  ```$ git clone https://github.com/foosel/OctoPrint.git```
2. Change into the OctoPrint folder: ```$ cd OctoPrint```
3. Request pip to run the setup file, i.e install all the necessary dependencies: ```$ sudo pip2.7 install .``` or ``` $  sudo pip2 install . ```
4. After installing OctoPrint, you should run it using: ```$ octoprint serve```   
5. Now you should verify your installation opening a web browser and going to http://localhost:5000
6. If everything went right you should see the OctoPrint home screen  

PS: In case you encountered a conflict error with PyYAML or any other package on step ```3.``` make sure you remove the previous installation from PyYAML/any other package from your computer by doing: 

 ```$ sudo rm -rf /usr/local/lib/python2.7/dist-packages/yaml ``` or ```$ sudo rm -rf /usr/local/lib/python2.7/dist-packages/THENAMEOFTHEPACKAGE ```

 and

```$ sudo rm -rf /usr/lib/python2.7/dist-packages/yaml ``` or ```$ sudo rm -rf /usr/lib/python2.7/dist-packages/THENAMEOFTHEPACKAGE ``` 

And then calling the setup file again to install all the dependencies but ignoring PyYAML (this is necessary because PyYAML is usually not completely deleted.) **Note that this would work for any other package from Python that is conflicting with the installation, i.e we could replace PyYAML for the package causing you trouble.**

 ```$ sudo pip2.7 install . --ignore-installed ${PyYAML} ``` or ```$ sudo pip2.7 install . --ignore-installed ${THENAMEOFTHEPACKAGE} ```

**_By entering the OctoPrint home screen for the first time you should setup your 3D Printer using the wizard from Octoprint, make sure to setup it correctly._** Don't forget to include the baudrate from your 3D Printer and enable the API Key **(respectively copying it)**.

**Paste the copied API Key on the 13th line of** ``` messenger.py ```

In case the printer you're using is a MakerBot, then you need to install the GPX plug-in, because OctoPrint doesn't support the x3g format, only gcode. GPX will take care of the conversion from gcode to x3g. Do the following:
1. In the octoPrint home screen go to configurations/Plugin manager/Get More
2. Search for GPX and click install 
3. After GPX is installed, make sure to check in the plugin's list if the GPX plugin is enabled
4. Make sure to configure correctly in the GPX plugin all the settings for your 3D printer (machine, gcode flavor and other settings)

## Installation

This project should be run from source, to do so just go to your ROS workspace (supposing it's called ``` catkin_ws ```):

``` $ cd ~/catkin_ws/src ```

And clone it with: 

``` $ git clone https://github.com/lar-deeufba/firos_octoros.git```

Tether octoROS with your catkin workspace:

``` $ cd ~/catkin_ws ```

``` $ catkin_make ```

Make ``` octoROS.py ``` executable so ROS can identify it:

``` $ roscd octo_ros/src ```

``` $ chmod +x octoROS.py ```

Add small function to open the port and run the OctoPrint server at once:

``` $ cd ~/ ```

``` $ echo "run_octoprint() { sudo chmod +666 /dev/ttyACM0 ; octoprint serve ; }" >> .bashrc  ```

## Usage
In order to use OctoROS, you need to have your Octoprint server running, have the wished model uploaded to it and located on ``` /src ``` folder from OctoROS:

First run the Octoprint server along with opening the port for the printer (MAKE SURE THE PRINTER IS CONNECTED TO THE COMPUTER!)

``` $ run_octoprint ```

Check if OctoROS has been succesfully installed by doing:

``` $ roslaunch octo_ros connect_to_printer.launch ```

The output should be like this (meaning that OctoROS Server is waiting for a goal (part to be printed)):

``` started core service [/rosout]
    process[printerWatcher-2]: started with pid [11920]
    /usr/local/lib/python2.7/dist-packages/requests/__init__.py:83: RequestsDependencyWarning: Old version of cryptography ([1, 2, 3]) may cause slowdown.
    warnings.warn(warning, RequestsDependencyWarning)
    [INFO] [1575918920.968175]: ---- OctoROS Initialized! ----
```

To integrate the Task Manager with OctoROS, please visit this page and follow the instructions from ``` README ```:

https://gitlab.com/hpoleselo/printpartskill


## Authors

* **Daniel Mascarenhas** - *Initial work* - [ielson](https://github.com/ielson)
* **Henrique Poleselo** - *Additional functionalities* - [hpoleselo](https://github.com/hpoleselo)

See also the list of [contributors](https://github.com/ielson/octoROS/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

Many thanks to the octoprint team, that made this awesome software
