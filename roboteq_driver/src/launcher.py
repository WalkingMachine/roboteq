#!/usr/bin/env python
import roslaunch
from os import listdir
from rospy import is_shutdown, sleep

launchPath = '../launch/drive.launch'
devPath = '/dev/SARA/drives/'
launchPortBase = 'port:='+devPath
nameBase = 'name:='

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

while not is_shutdown():
    try:
        for name in listdir(devPath):
            print("name:="+str(name))

            cli_args = [launchPath, launchPortBase+name, nameBase+name]
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

            parent.start()

            break
    except:
        print('No roboteq drive connected. Checking every 5 seconds.')
        sleep(5)
