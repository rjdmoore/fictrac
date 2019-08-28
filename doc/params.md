This document provides a brief overview of FicTrac's configuration parameters. Many of these parameters are set automatically by the configuration utility. See the main documentation for instructions on how to [configure FicTrac](../README.md#Configuration) before use.

In the table below, the various possible parameters are listed. If nothing is listed under `Default value` then the parameter must be specified by the user. The valid range may use [interval notation](https://en.wikipedia.org/wiki/Interval_(mathematics)). The `Should I touch it?` column should give you some idea of which params to play around with.

| Param name | Param type | Default value | Valid range | Should I touch it?  | Description |
|------------|------------|---------------|-------------|---------------------|-------------|
| src_fn     | string OR int |            | int=\[0,inf) | Yes, you have to   | A string that specifies the path to the input video file, OR an integer that specifies which of several connected USB cameras to use. Paths can be absolute or relative to the working directory. |
| vfov       | float      |               | (0,inf)     | Yes, you have to    | Vertical field of view of the input images in degrees. |
|            |            |               |             |                     |             |
| do_display | bool       | y             | y/n         | If you want to      | Display debug screen during tracking. Slows execution very slightly. |
| save_debug | bool       | n             | y/n         | If you want to      | Record the debug screen to video file. Note that if the source frame rate is higher than FicTrac's processing frame rate, frames may be dropped from the video file. |
| save_raw   | bool       | n             | y/n         | If you want to      | Record the input image stream to video file. Note that if the source frame rate is higher than FicTrac's processing frame rate, frames may be dropped from the video file. |
| sock_port  | int        | -1            | (0,inf)     | If you want to      | Socket port over which to transmit FicTrac data. If unset or < 0, FicTrac will not transmit data over sockets. |
| com_port   | string     |               |             | If you want to      | Serial port over which to transmit FicTrac data. If unset, FicTrac will not transmit data over serial. |
| com_baud   | int        | 115200        |             | If you want to      | Baud rate to use for COM port. Unused if no com_port set. |
|            |            |               |             |                     |             |
| fisheye    | bool       | n             | y/n         | Only if you need to | If set, FicTrac will assume the imaging system has a fisheye lens, otherwise a rectilinear lens is assumed. |
| q_factor   | int        | 6             | (0,inf)     | Only if you need to | Adjusts the resolution of the tracking window. Smaller values correspond to coarser but quicker tracking and vice-versa. Normally in the range [3,10]. |
| src_fps    | float      | -1            | (0,inf)     | Only if you need to | If set, FicTrac will attempt to set the frame rate for the image source (video file or camera). |
| max_bad_frames | int    | -1            | (0,inf)     | Only if you need to | If set, FicTrac will reset tracking after being unable to match this many frames in a row. Defaults to never resetting tracking. |
| opt_do_global | bool    | n             | y/n         | Only if you need to | Perform a slow global search after max_bad_frames are reached. This may allow FicTrac to recover after a tracking fail, but should only be used when playing back from video file, as it is slow! |
| opt_max_err | float     | -1            | \[0,inf)    | Only if you need to | If set, specifies the maximum allowable matching error before declaring a bad frame (i.e. tracking fail). Matching error is printed to screen during tracking (err=...), and also output in the [data file](doc/data_header.txt) (delta rotation error score). If unset, FicTrac will never detect bad matches (tracking will fail silently). |
| thr_ratio  | float      | 1.25          | (0,inf)     | Only if you need to | Adjusts the adaptive thresholding of the input image. Values > 1 will favour foreground regions (more white in thresholded image) and values < 1 will favour background regions (more black in thresholded image). |
| thr_win_pc | float      | 0.2           | \[0,1]      | Only if you need to | Adjusts the size of the neighbourhood window to use for adaptive thresholding of the input image, specified as a percentage of the width of the tracking window. Larger values avoid over-segmentation, whilst smaller values make segmentation more robust to illumination gradients on the trackball. |
| vid_codec  | string     | h264          | [h264,xvid,mpg4,mjpg,raw] | Only if you need to | Specifies the video codec to use when writing output videos (see `save_raw` and `save_debug`). |
| sphere_map_fn | string  |               |             | Only if you need to | If specified, FicTrac will attempt to load a previously generated sphere surface map from this filename. |
|            |            |               |             |                     |             |
| opt_max_evals | int     | 50            | (0,inf)     | Probably not        | Specifies the maximum number of minimisation iterations to perform each frame. Smaller values may improve tracking frame rate at the risk of finding sub-optimal matches. Number of optimisation iterations is printed to screen during tracking (its=...). |
| opt_bound  | float      | 0.35          | (0,inf)     | Probably not        | Specifies the optimisation search range in radians. Larger values will facilitate more track ball rotation per frame, but result in slower tracking and also possibly lead to false matches. |
| opt_tol    | float      | 0.001         | (0,inf)     | Probably not        | Specifies the minimisation termination criteria for absolute change in input parameters (delta rotation vector). |
|            |            |               |             |                     |             |
| c2a_cnrs_xy | vec\<int> |               |             | Set by ConfigGui    | Specifies the corners {X1,Y1,X2,Y2,...} of a square shape aligned with the animal's XY axes. Set interactively in ConfigGUI. |
| c2a_cnrs_yz | vec\<int> |               |             | Set by ConfigGui    | Specifies the corners {X1,Y1,X2,Y2,...} of a square shape aligned with the animal's YZ axes. Set interactively in ConfigGUI. |
| c2a_cnrs_xz | vec\<int> |               |             | Set by ConfigGui    | Specifies the corners {X1,Y1,X2,Y2,...} of a square shape aligned with the animal's XZ axes. Set interactively in ConfigGUI. |
| c2a_src    | string     |               |             | Set by ConfigGui    | Specifies which of the above corner sets is used to compute the camera-animal transform. Set interactively in ConfigGUI. |
| c2a_r      | vec\<float> |              |             | Set by ConfigGui    | Rotational component of the camera-animal transform. Computed automatically by ConfigGUI. |
| c2a_t      | vec\<float> |              |             | Set by ConfigGui    | Translational component of the camera-animal transform. Computed automatically by ConfigGUI. |
| roi_circ   | vec\<int>  |               |             | Set by ConfigGui    | Specifies points {X1,Y1,X2,Y2,...} around the circumference of the trackball in the input image. Set interactively in ConfigGUI. |
| roi_c      | vec\<float> |              |             | Set by ConfigGui    | Camera-frame vector describing the centre point of the trackball in the input image. Computed automatically by ConfigGUI. |
| roi_r      | float      |               |             | Set by ConfigGui    | Half-angle describing the radius of the trackball in the input image. Computed automatically by ConfigGUI. |
| roi_ignr   | vec<vec\<int>> |           |             | Set by ConfigGui    | Specifies possibly several polygon regions {{X11,Y11,X12,Y12,...},{X21,Y21,X22,Y22,...},...} that should be ignored during matching (e.g. where the animal obscures the trackball). Set interactively in ConfigGUI. |
