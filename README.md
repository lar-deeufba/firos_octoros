## OctoROS-FIROS Interface
This provides the functional interface with OctoROS and FIROS.

### Installation

**Before you proceed with any of these instructions make sure to go individually to each folder ([```firos```])(https://github.com/lar-deeufba/firos_octoros/tree/master/firos) and ([``` octoros_adapted ```])(https://github.com/lar-deeufba/firos_octoros/tree/master/octoros_adapted) and follow the installation instructions!**

Make sure to clone everything to your workspace, supposing it's ``` catkin_ws ```. Go to the root of your ``` catkin_ws ``` then make it:
- ``` $ cd ~/catkin_ws ```

- ``` $ catkin_make ```

Now source your workspace:

- ``` $ source devel/setup.bash ```


### Usage
Follow the installation instructions separately for each folder and when everything is done follow the following steps to print a part and send the data to the broker:

1. Run the Octoprint interface:

	``` $ run_octoprint ```
		
2. ``` $ print_file.sh YOURMODEL.gcode```

3. ``` $ roslaunch firos firos.launch ```

4. To check if OctoROS is retrieving the data from the printer, do the following:

	``` $ rostopic echo /printer3d ```

### Expected result

