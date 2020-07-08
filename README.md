#######################################
Vorpal Hexapod and Gamepad Code V3
#######################################

This is the V3 revision of the Vorpal Hexapod Robot and Gamepad code.

If you are currently running an older release, you must load both the gamepad and robot code to get the new functions. You can't load one without the other and expect
it to work.

This V3 release has the following enhancements:

1) Double Click Mode. The W, D, and F rows of the gamepad can now be double clicked to access new functions as follows:

W1W1: Wave Gait (one leg at a time walking)
W2W2: Belly Crawl gait.
W3W3: Quadruped gait (uses four legs and sways middle legs)
W4W4: Changes direction by redefining where the front of the robot is. F and B dpad buttons do the same as usual. L moves as if the front of the robot is between leg
      0 and leg 1. R moves as if the front of the robot is between legs 5 and 4. B reverses the sense of the front of the robot.

D1D1: Twitch dancing.
D2D2: Sway dancing.
D3D3: Star dancing (star patterns with legs)
D4D4: Brownian motion dancing (sort of like "the worm" from Animal House)

F1F1: Dog Beg mode. Does various tricks: "beg for food", wave goodbye, queen's wave, wave with both hands.
F2F2: The dpad controls just leg 5, allowing it move up, down, left, or right.
F3F3: The dpad controls just leg 0, like F2F2.
F4F4: Raise/lower the robot with and without spiral motions.

2) Leg safety mode. The robot will now briefly put all six legs on the ground when switching between most modes. This avoids motor stresses that occur, for example, when
   going from "Mr. Hands Mode" (D4) to Ninja Mode (F1). Modes that retain state (such as F3, F4, F3F3, etc.) do not reset to all six legs standing, allowing you to stack
   up state-holding sequences. For example you could use F2F2 to move just leg 5 to a new position, then click F3F3 to move just leg 0, and your leg 5 moves will not be
   affected.

3) Smoother motions for F3, F4, F2F2, F3F3, and other state-holding modes. The motions used to be fairly jerky but now each time slice while waiting for new
   gamepad packets interpolates where the leg is moving toward.

4) "Terse mode" radio control protocol allows access to all double click functions. Double clicks on W can be accessed using X, D using Y and F using Z. This is mainly
   used by people who are creating alternative gamepads or gamepad apps, or want to control the robot using an off the shelf gamepad that supports bluetooth and has
   definable sequences for different button pushes.

5) There is now a master timing control #define near the start that makes the leg motions move at appropriate speeds for different sized hexapods (i.e. megapod and
   gigapod). While the megapod worked reasonably well under the same timing as the small version of the robot and on a tiny difference is there, the gigapod
   needed significant different timing.

LIMITATIONS

The new double click leg motions are not available to the ScratchX interface yet. We are doing a major overhaul of our Scratch code in 2020.

# VorpalHexapodArduino
Arduino code for both the Vorpal Hexapod and Vorpal Hexapod Gamepad.

This is the Arduino IDE code for the current version of the Vorpal Hexapod Project.

To compile it you need the Adafruit 16 Channel Servo Controller Library and the SDFAT library.

The libraries we've tested with may be found in a public dropbox folder: tinyurl.com/VORPALFILES

PROJECT WIKI
-----------------------------
Please see the project wiki on our website for complete details including how to build the robot, where to download the libraries, how to use the robot, games and activities, technical information, and more:

Main Hexapod Page: http://vorpalrobotics.com/wiki/index.php?title=Vorpal_The_Hexapod
Assembly Instructions: http://vorpalrobotics.com/wiki/index.php?title=Vorpal_The_Hexapod_Building_Instructions
Arduino Libraries and other files: http://vorpalrobotics.com/wiki/index.php?title=Vorpal_Hexapod_Source_Files

The current 3D model files are posted on Thingiverse.com, search for "Vorpal Hexapod" and you'll find it. There are Things there for the robot, gamepad, and each activity and accessory.

SOURCING THE ELECTRONICS
------------------------

You are welcome to source all the parts yourself, but we respectfully ask you to consider buying some or all of the parts from our store:
http://store.vorpalrobotics.com

Buying our kits cuts at least 1.5 to 2 hours off the assembly time because we give you all the electronics pre-soldered, bluetooth modules are pre-configured to pair with each other automagically, and of course you know you're getting all compatible parts that work with the 3D models perfectly.

We have both kits and also each part individually so you can pick and choose what you source yourself and what you source through our store.

VORPAL FORUM
------------
You can join the community of Vorpal Hexapod enthusiasts to discuss Vorpal the Hexapod by signing on to our Forum here:
https://groups.google.com/forum/#!forum/vorpal-robotics-forum

VORPAL MAILING LIST
-------------------
Our monthly newsletter has news and information about Vorpal the Hexapod and other Vorpal Robotics projects.

Sign up for our newsletter here: http://eepurl.com/dcJCgr
