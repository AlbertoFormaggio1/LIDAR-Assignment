# LIDAR Scanner

## Assignment
This assignment requires you to develop a C++ module for handling the incoming data from a LIDAR. A LIDAR is a sensor capable of making distance measurements using a
laser beam. The principle of operation is similar to the one of a laser meter, but in the case of the lidar the laser beam is rotated and the measurement is made at regular angle intervals. A LIDAR therefore scans a plane in space and reports the distance measurements detected on that plane at regular intervals.

You can find more information about how a LIDAR works here: https://it.wikipedia.org/wiki/Lidar

This sensor generates a data flow consisting of a set of ```double``` representing the readings at various angles. 
The angle that separates two consecutive readings is called __angular resolution__ and it is a data intrinsic data of each LIDAR chosen at the moment of its construction. Two different LIDARs can have different angular resolutions.

__Example 1__: A LIDAR with 1° angular resolution can measure the following distances
```
| Angle | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Distance | 1.01 | 1.10 | 1.65 | 1.78 | 2.22 | 0.88 | 0.92 | 1.73 | 1.90 |
```

Unfortunately, this homework was for an Italian course of Programming so both the instructions and the comments are in Italian.
