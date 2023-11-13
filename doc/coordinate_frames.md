This document explains the various coordinate frames used by the FicTrac system. The coordinate frames are illustrated in the figure below.

![01-treadmill_diagram-label-fixed](https://github.com/rjdmoore/fictrac/assets/3844483/aa9e08ea-dec9-44bf-9c18-01df0d823ad0)

Note that this illustration is nearly identical to Fig. 1 from [Moore et al, 2014](https://doi.org/10.1016/j.jneumeth.2014.01.010), except here we have flipped the orientation of the "world" axis Wxyz on which the fictive path is drawn, because it was potentially misleading in the original version.

All coordinate frames form valid [right-handed coordinate systems](https://en.wikipedia.org/wiki/Right-hand_rule).

### Camera coordinate frame
The camera coordinate frame is defined by:
* +X camera-right (increasing cols)
* +Y camera-down (increasing rows)
* +Z out of the camera

FicTrac inherently measures rotation of the tracking ball in the camera coordinate frame. Rotations need to be transformed into the animal coordinate frame to decode animal motions.

Ball rotations in the camera coordinate frame are output in columns 2-4.

### Animal/lab coordinate frame
The animal, or lab, coordinate frame is defined by:
* +X animal-forward
* +Y animal-right
* +Z animal-down

The [SO(3)](https://en.wikipedia.org/wiki/3D_rotation_group) rotational transformation between the camera and animal/lab coordinate frames is computed during the configuration procedure, and is used to transform ball rotations measured in the camera coordinate frame into rotations in the animal coordinate frame.
Once the ball rotations are known in the animal's coordinate frame, they can be mapped to animal motions:
* Clockwise rotation about +X axis &rarr; animal sidestepping left
* Clockwise rotation about +Y axis &rarr; animal walking forwards
* Clockwise rotation about +Z axis &rarr; animal turning left

Ball rotations in the animal/lab coordinate frame are output in columns 6-8.

### World (fictive path) coordinate frame
The world coordinate frame, in which the fictive path is generated, is defined by:
* +X north (initial animal heading direction)
* +Y east (initial animal-right direction)
* +Z down (animal-down)

Animal forward motion in the absence of any change in heading direction or sidestepping will give fictive motion in the +X direction, and sidestepping motion to animal-right in the absence of other rotations will give fictive motion in the +Y direction. Equivalently, if the animal turns clockwise 90 degrees and walks forwards this will give fictive motion in the +Y direction.

The animal's fictive position is output in columns 15-16 and the animal's integrated heading direction is output in column 17.

### Plotting the fictive path
Plotting the fictive path (cols 15-16 in the output data) directly on a normal cartesian plane will result in the animal's initial heading direction being aligned with the right (+X) axis and initial animal-right being aligned with the up (+Y) axis.
This will mean that clockwise turning motion of the animal will appear anti-clockwise on the plot.

To correctly plot the fictive path (cols 15-16) on a cartesian plane, simply switch X and Y during plotting, i.e. plot(Y,X) instead of plot(X,Y). This will make the initial heading direction of the animal aligned with the up (+Y) axis, and initial animal-right aligned with the right (+X) axis.

### Interpreting the configImg
The configured animal/lab axes are displayed in the config image (cropped example below). It is important to check that these axes appear correct w.r.t. the position/orientation of the animal. If the axes do not look correct, then you must re-run the configuration program.

![image](https://github.com/rjdmoore/fictrac/assets/3844483/a5deb4cd-7a36-4010-a60f-8817eae1df39)

To help with disambiguating the orientation of the axes, they are drawn with "into the page" / "out of the page" notation - i.e. a circle with a dot to denote "out of the page" and a circle with a cross to denote "into the page" (a fun mnemonic is that x marks the spot to dig into the page). In the above sample config image, the directions of the X and Y axes are fairly unambiguous but there are two configurations for the Z axis that appear the same in 2D: a left-handed axes with Z coming out of the page towards the viewer, and a right-handed axes with Z going into the page and aligned with the animal's down axis. All axes should be right-handed axes, and additionally we know that Z should be aligned with animal down, but the circle/cross notation just helps to confirm that the axes have the correct orientation.
