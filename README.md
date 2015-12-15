robotics-fall-2015
==================

A project by [Martin Bramlett](https://github.com/bramlemk) and [Danny McClanahan](https://github.com/cosmicexplorer).

[Arduino code](arduino/keyboard) is available, and can be run by uploading the [keyboard.ino sketch](arduino/keyboard/keyboard.ino) to most Arduino devices. We used a Duemilanove, but it shouldn't affect anything if you use something else. The Arduino code listens to connections over digital input pins 0 through 7, and when the circuit is closed over any of those pins, emits a tone corresponding to that pin's frequency through digital pin 11. This digital pin 11 can be hooked up directly to a pair of speakers; I recommend using old ones, though, since it is a pure square wave. The Arduino is unfortunately monophonic due to constraints on the design.

To run the robot code, run the command:

``` shell
rosrun pianist demo3.py
```

The running prompt will display some diagnostic information, and should prompt you for a calibration point, and then a song file; it should default to an existing song if you just press enter. Calibration currently takes some skilled human effort, and methods to improve it are discussed in the paper.
