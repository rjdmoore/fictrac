This document provides a detailed description of the FicTrac algorithm as well as requirements and recommendations for getting the best performance from the software for your particular setup. You will also find a description of the input parameters and data output by the software, along with a detailed description of the configuration procedure.

## FicTrac overview

FicTrac tracks the orientation of a patterned sphere by matching the currently visible portion of the sphere againt a map of the entire surface. An *a priori* map does not have to be provided (but can be), as FicTrac can learn the map from scratch online by piecing together successive and overlapping views of the sphere's surface. Internally, FicTrac maintains a model sphere, whose surface is the learned map. Each frame, FicTrac computes the orientation of the model sphere that best matches the current perspective of the real sphere using a non-linear optimisation procedure. Each candidate orientation is ranked by computing the image difference between the model view and real view of the sphere. The most probable orientation is the one that corresponds to the least image difference. To reduce the unwanted effects of changing lighting conditions on the matching process, the input region of interest around the sphere is first thresholded to give a binary representation (black/white). 

Once the best match for a particular frame is found, the sphere model surface map is updated with the current view. Due to small matching or thresholding errors, an individual pixel in the surface map might not initially be allocated the correct colour (black/white), but over time, as that particular pixel is revisited frame after frame, FicTrac builds up a more confident representation of its true colour. In this way, the model sphere's surface map iteratively approaches the appearance of the true sphere.

Knowing the orientation of the sphere is only half the problem. In order to extract some useful information about the movements of the animal that is rotating the sphere, we must transform the rotations observed in the camera's frame of reference to rotations about the animal's cardinal axes - forward, right, and down. This gives the side-stepping speed, forward speed, and yaw rate (turning speed) of the animal respectively. By integrating these values, we can obtain the fictive 2D path of the animal over time - as if it were walking on a flat piece of paper.

FicTrac can provide more accurate data than an approach based on optical mouse sensors because FicTrac inherently computes the absolute orientation of the sphere, which will not drift over time, whereas optical mouse sensors placed 90 deg apart on the circumference of the sphere will compute the local surface velocity. The difference is that FicTrac's estimate for the rotational velocity of the sphere should be zero-mean, whereas an estimate derived from optical mouse sensors could be subject to sensor biases. However, there are three important points to note:
    
 * The above is true for the rotational velocity of the sphere, however typically it is the fictive path of the animal that is sought after. To obtain the fictive path, the components of the sphere's rotational velocity must be integrated each frame - because rotations of a sphere are [non-commutative](https://en.wikipedia.org/wiki/Commutative_property). This means that both FicTrac's estimate of the fictive 2D path as well as an estimate derived from optical mouse sensors will drift from the true path over time. However, since FicTrac's estimate is derived from a velocity estimate with zero-mean noise, it should drift more slowly.
    
 * FicTrac inherently computes the absolute orientation of the sphere each frame. However, as noted in the above point, to compute the fictive path, the change in orientation between frames (i.e. rotational velocity) must be integrated. Thus, to obtain the rotational velocity, FicTrac must effectively take the derivative of the absolute orientation vector, which produces a noisy (although zero-mean noise) estimate of the sphere's rotational velocity. Conversely, optical mouse sensors directly compute surface velocity of the sphere and so should provide lower noise estimates of the sphere's rotational velocity. Furthermore, FicTrac operates at frame rates up to several hundred FPS, whereas optical mouse sensors inherently operate at frame rates up to several *thousand* FPS - so additional filtering can be applied to signals derived from optical mouse sensors.
    
 * Finally, and perhaps most importantly, both FicTrac and approaches based on optical mouse sensors inherently measure the orientation or rotation of the sphere in the *sensor's frame of reference*. In order to extract useful information on the movements of the animal, the computed rotational velocity of the sphere must first be transformed to the animal's frame of reference. If this transformation is not performed very accurately, then the component of the sphere's rotation attributed to the animal's change in heading will be incorrect, so the animal's heading direction will be incorrectly integrated, and thus the computed fictive 2D path will quickly diverge from the true path (the animal's side- and forward-stepping components will also be incorrectly estimated but these will have a lesser impact on the accuracy of the fictive path). For approaches based on optical mouse sensors, this transformation to the animal's coordinate frame is performed implicitly with the placement of the animal - the animal is placed exactly at the "north pole" of the track ball and the optical mouse sensors are placed around the "equator". With FicTrac, the camera may be placed freely within the experimental enclosure, and so this transformation must be estimated. A configuration utility (configGui) is provided with FicTrac that allows the transformation to be computed interactively from the corners of a square shape placed within the field of view of the camera. **It is important that this square shape is properly aligned with the animal's axes and that the configuration process is followed as carefully as possible to avoid introducing biases to estimates of the animal's motions.**
 
FicTrac's main advantages over an approach based on optical mouse sensors are that it reduces the time, cost, and effort required to set up a new experimental enclosure by removing the need to design and build custom electronics, mounts, and processing software to support optical mouse sensors. FicTrac requires only a camera and computer, which are both often included in experimental setups anyway, and because the camera can be placed freely in the experimental enclosure, it reduces clutter around the animal. Additionally, FicTrac's matching scheme provides visual as well as quantitative feedback on the performance of the algorithm and quality of the produced data - allowing issues to be identified quickly.

FicTrac is quick to set up and easy to use, so can be quickly evaluated for any experimental setup.

## Hardware requirements

### Track ball

### Video camera

### PC/laptop

### Lighting

## Configuring FicTrac

## Input parameters

## Output data

## Failure modes / troubleshooting

