roboteq
=======

https://travis-ci.org/WalkingMachine/roboteq.svg?branch=master

ROS VERSION: [Kinetic](http://wiki.ros.org/kinetic)

ROS driver for serial Roboteq motor controllers. This driver is suitable for use with Roboteq's
Advanced Digital Motor Controllers, as described in [this document][1]. Devices include:

Brushed DC: HDC24xx, VDC24xx, MDC22xx, LDC22xx, LDC14xx, SDC1130, SDC21xx  
Brushless DC: HBL16xx, VBL16xx, HBL23xx, VBL23xx, LBL13xx, MBL16xx, SBL13xx  
Sepex: VSX18xx

The node works with a MicroBasic script in the drive, which then publishes ASCII sentences at 10Hz, 25Hz and 50Hz with the data corresponding to the Status, ID and per-channel Feedback messages published by the driver.

Before ROS file are fully launch, the node will first try to connect with the drive to get the ID of the drive. ROS won't continue until it has an ID from the drive.
When the drive give the `unique` ID, ROS will create multiple topics like :
```
/drive1/cmd1
...
/drive3/status3
```
You can identified a drive like by looking at the id : `/drive<ID>/<topics><ID>`

### How to
1. You have to physicaly identified each drive;
2. Then on each drive you have to :
  * load the default drive profile
  * load the script and identified the default ID of your drive
3. (Optional) if there is any power connected to the drive, you may need to do a power reset (off/on);
4. Connect the USB port from the drive to the computer (any port);
5. Run `roslaunch roboteq_driver driveUltime.launch` (this will start all the drive);
  * If you want to run drive by drive you can : `roslaunch roboteq_driver drive<Count>.launch`
6. Now all the drive are connected! Do what you must!

[1]: https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file
