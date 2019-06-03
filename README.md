## OctoROS-FIROS Interface
This provides the functional interface with OctoROS and FIROS.

### Setup

**Before you proceed with any of these instructions make sure to go individually to each folder (``` firos ``` and ``` octoROS ```) and follow the installation and test instructions!**

Make sure to clone everything to your workspace, supposing it's ``` catkin_ws ```. Go to the root of your ``` catkin_ws ``` then make it:
- ``` $ cd ~/catkin_ws ```

- ``` $ catkin_make ```

Now source your workspace:

- ``` $ source devel/setup.bash ```


### Usage
Follow the installation instructions separately for each folder and when everything is done follow the following steps to print a part and send the data to a broker:

1. Run the Octoprint interface:

	``` $ octoprint serve ```

2. Open your browser and enter on the following link: http://localhost:5000 

3. Connect the printer to your computer and open the port properties for the connected 3D Printer:

	```$ sudo chmod 666 /dev/ttyACM0```
		
4. Go to the browser interface and click on the button ```Connect```. If you can change the temperatures from the extruder or the bed, then the connection has been successfull.

5. Run ROS: (We leave ROS running because than we can fire the printing when we want)

    ``` $ roscore ```

6. On another terminal we run the core for FIROS:

	``` $ roslaunch firos core.launch ```


7. On another terminal we run the sender for our printer:

	``` $ roslaunch firos sender.launch ```

8. Open up another terminal and run OctoROS (it's going to start to print the part)

	``` $ roslaunch octo_ros connect_to_printer.launch ```


9. To check if OctoROS is retrieving the data from the printer we do:

	``` $ rostopic echo /printer3d ```

10. You can compare the step on ``` 9.``` with the step on ``` 7. ``` both should be refreshed at the same rate (```sender.py``` takes the published topic ```/printer3d``` and resends to the broker because we have our own message type so we resend it in a form of JSON.)

### Expected result

### Debug tools
