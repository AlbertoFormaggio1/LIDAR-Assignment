# LIDAR Scanner
<p align="center">
    <img src="https://www.unidformazione.com/wp-content/uploads/2018/04/unipd-universita-di-padova.png" width="250" alt="University of Padua"/>
</p>

## Assignment
This assignment requires you to develop a C++ module for handling the incoming data from a LIDAR. A LIDAR is a sensor capable of making distance measurements using a
laser beam. The principle of operation is similar to the one of a laser meter, but in the case of the lidar the laser beam is rotated and the measurement is made at regular angle intervals. A LIDAR therefore scans a plane in space and reports the distance measurements detected on that plane at regular intervals.

You can find more information about how a LIDAR works here: https://it.wikipedia.org/wiki/Lidar

This sensor generates a data flow consisting of a set of ```double``` representing the readings at various angles. 
The angle that separates two consecutive readings is called __angular resolution__ and it is a data intrinsic of each LIDAR chosen at the moment of its construction. Two different LIDARs can have different angular resolutions.

__Example 1__: A LIDAR with 1° angular resolution can measure the following distances

| Angle [°] | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Distance [m] | 1.01 | 1.10 | 1.65 | 1.78 | 2.22 | 0.88 | 0.92 | 1.73 | 1.90 |

Since the angular resolution is a constant value, the data flow of that LIDAR is simply the succession of the various distance readings, then the data corresponding to the measurements in the table are the following:
1.01 1.10 1.65 1.78 2.22 0.88 0.92 1.73 1.90

The LIDARs you have to manage have a 180 ° angle of view, therefore a sensor with a resolution of 1° provides **181** values for each scan, a sensor with a resolution of 0.5° provides 361 values for each scan, a sensor with a resolution of 0.25 ° provides 721 values for each scan, etc. 
The laser scanners managed can have a resolution between 1 ° and 0.1 °.


## Compile and run

To compile

```
g++ -o main *.cpp
```

To run the program write and the terminal:
```
./main
```
And press Enter.

__WARNING__ The input text data must be in the same folder as the source code in order for the program to work


Unfortunately, this homework was for an Italian course of Programming so both the documentation and the comments are in Italian.
