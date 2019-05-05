<div align="left">
  <img src="http://rjdmoore.net/fictrac/header_text.jpg"><br><br>
</div>

**FicTrac** is an open-source software library for reconstructing the fictive path of an animal walking on a patterned sphere. The software is fast, flexible, easy to use, and simplifies the setup of closed-loop tracking experiments.

FicTrac was originally developed by researchers at the [Queensland Brain Institute](http://qbi.uq.edu.au/) at the University of Queensland, Australia for tracking honeybees and fruit flies during closed-loop tethered walking experiments, but it has since proved useful for tracking a wide range of animals with different movement speeds, track ball diameters and patterns, and stimuli.

On this page you'll find information for:
* [Getting started](#getting-started) - including [hardware requirements](#hardware-requirements), and basic steps for [installing](#installation), [configuring](#configuration), and [running FicTrac](#running-fictrac).
* [Research](#research) - including works to cite and links to the original FicTrac publication and preprint (pdf).
* [Contributing](#contribution-guidelines) - how to go about making your fixes, additions, and customisations available for other users.
* [Re-use](#license) - licensing information, if you plan on using FicTrac or the source code.

You might also be interested in the following links:
* [Demo video](http://youtu.be/BeGYOEOdWjw) - Quick (30s) overview of what FicTrac does and how it works.
* [FicTrac manual](doc/requirements.md) - Detailed instructions, description of output data, parameters, recommendations, etc.
* [Homepage](http://fictrac.rjdmoore.net) - Contact details for the main author/developer, links, and further info.
* [Forum](http://www.reddit.com/r/fictrac/) - Subreddit for faqs, support, advice, discussions, etc.
* [Mailing list](http://fictrac.rjdmoore.net/mail.html) - Subscribe to receive important announcements and updates.

Happy tracking!

## Getting started

If you're just setting up your lab, or wondering whether FicTrac is suitable for your setup (spoiler: yes, probably), check the [hardware requirements section below](#hardware-requirements) for the basic requirements.

If you already have an experimental enclosure with a camera, you can use FicTrac to either process recorded videos offline or to run live from the camera. Skip ahead to [install](#installation), [configure](#configuration), and [run FicTrac](#running-fictrac).

### Hardware requirements

Very briefly, FicTrac imposes almost no special requirements on your experimental setup, other than that a pattern must be applied to the track ball. However, there are a number of tips that can help you get the best results from using FicTrac.

A typical FicTrac experimental setup might include at least the following equipment:
* *Animal tether/harness* - for keeping the animal stationary on the track ball
* *Stimulus* - sensory feedback for the animal
* *Track ball support* - structure to hold track ball in place whilst allowing free rotation
* [Track ball](doc/requirements.md#track-ball) - lightweight sphere that is rotated by the animal's walking/turning motions. Surface pattern should ideally be high-contrast, non-repeating, non-reflective (not glossy), and contain around 10~50 variously sized blobby shapes.
* [Video camera](doc/requirements.md#video-camera) - for monitoring the track ball (and animal). Resolution is not important, and for vertebrates a USB webcam is likely sufficient. For faster moving insects, the camera should support frame rates >100 Hz. In all cases, the frame rate, exposure, and lens should be chosen to ensure the track ball pattern is well-exposed under all lighting/stimulus conditions and that there is no motion blur during the fastest expected movements. At least one half of one hemisphere of the track ball surface should be visible by the camera.
* [PC/laptop](doc/requirements.md#pclaptop) - for running FicTrac software (and generating closed-loop stimuli). Processor should be somewhat recent (>2 GHz, multi-core).
* [Lighting](doc/requirements.md#lighting) - ambient lighting should be diffuse (no specular reflections from track ball surface) and bright enough to give good track ball surface exposure at chosen frame rate.

FicTrac imposes no requirements on the *italicised* items; how you design these is completely dependent on other factors.

### Installation

[![Build Status](https://dev.azure.com/rjdmoore/FicTrac/_apis/build/status/rjdmoore.fictrac?branchName=master)](https://dev.azure.com/rjdmoore/FicTrac/_build/latest?definitionId=1&branchName=master)

The FicTrac source code can be built for both Windows and Ubuntu (Linux) operating systems, or you can build and run FicTrac from within a [virtual machine](https://www.virtualbox.org/) on any operating system.

1. Download and install required dependencies:
    1. [Cmake build system](https://cmake.org/download/) (binary distribution)
    2. For Windows installations, if you don't already have Visual Studio (C++ workflow) installed, you will need to install the [build tools](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2017).
    3. Clone or download the [Vcpkg](https://github.com/Microsoft/vcpkg) repository and then follow the guide to install (make sure to perform the bootstrap and integration steps).
    4. Using Vcpkg, install OpenCV and NLopt software packages:
```
[Windows] .\vcpkg install opencv[ffmpeg]:x64-windows nlopt:x64-windows
[Linux] ./vcpkg install opencv[ffmpeg]:x64-linux nlopt:x64-linux
```
2. Clone or download the FicTrac repository, then navigate to that folder, open a terminal, and create a build directory:
```
mkdir build
cd build
```
3. Run Cmake to prepare the necessary build files for FicTrac. Here, we will need to provide the path to the Cmake toolchain file that was installed by Vcpkg (this path is printed to terminal when you run the Vcpkg system-wide integration step).
```
[Windows] cmake -G "Visual Studio 15 2017 Win64" -D CMAKE_TOOLCHAIN_FILE=C:\path\to\vcpkg\scripts\buildsystems\vcpkg.cmake ..
[Linux] cmake -D CMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake ..
```
5. Finally, build and install FicTrac:
```
[Windows] cmake --build . --config Release -j 4
[Linux] cmake --build . --config Release -- -j 4
```

If everything went well, the executables for FicTrac and a configuration utility will be placed under the `bin` directory in the FicTrac project folder.

Remember to update and re-build FicTrac occasionally, as the program is still under development and fixes and improvements are being made continuously.

#### USB3 camera installation

If you are using a USB3 camera and are receiving error messages when FicTrac tries to connect to your camera, you may need to tell FicTrac to use the SDK provided with your camera, rather than the generic OpenCV interface. The instructions for switching to the camera's SDK are different for each manufacturer. Currently there is support for PGR (FLIR) USB3 cameras via the Spinnaker SDK.

##### PGR (FLIR) Spinnaker SDK

1. Download and install the latest Spinnaker (full) SDK from [PGR downloads page](https://www.ptgrey.com/support/downloads).
2. When preparing the build files for FicTrac using Cmake, you will need to specify to use Spinnaker using the switch `-D PGR_USB3=ON` and depending on where you installed the SDK, you may also need to provide the SDK directory path using the switch `-D PGR_DIR=...`. For example, for a Windows installation you would replace step 3 above with:
```
cmake -G "Visual Studio 15 2017 Win64" -D CMAKE_TOOLCHAIN_FILE=<vcpkg root>/scripts/buildsystems/vcpkg.cmake -D PGR_USB3=ON -D PGR_DIR="C:\path\to\Spinnaker" ..
```
3. Follow the other build steps as normal.

Before running FicTrac, you may configure your camera (frame rate, resolution, etc) as desired using the SDK utilities.

### Configuration

There are two necessary steps to configure FicTrac prior to running the program:
1. You must provide a text file that contains important [configuration parameters](doc/params.md) for your setup. At a minimum, this config file must define the parameters `src_fn` and `vfov`, which define the image source (path to video file or camera index) and vertical field of view (in degrees) of your camera respectively. You will find an example config file in the `sample` directory.
2. You must run the interactive configuration program (configGui). This program will guide you through the configuration of the track ball region of interest within your input images and the transformation between the camera's and animal's frames of reference.

A more [detailed guide](doc/requirements.md) on how to configure FicTrac for your setup and an explanation of all the [configuration parameters](doc/params.md) can be found in the `doc` directory.

### Running FicTrac

To configure FicTrac for the provided sample data, simply open a terminal in the FicTrac project folder and type:
```
cd sample
[Windows] ..\bin\Release\configGui.exe config.txt
[Linux] ../bin/configGui config.txt
```
The sample config file `config.txt` is already configured for the sample data, but you can step through the configuration process to check that everything looks ok.

Then, to run FicTrac, type:
```
[Windows] ..\bin\Release\fictrac.exe config.txt
[Linux] sudo ../bin/fictrac config.txt
```

FicTrac will usually generate two output files:
1. Log file (*.log) - containing debugging information about FicTrac's execution.
2. Data file (*.dat) - containing output data. See [data_header](doc/data_header.txt) for information about output data.

The output data file can be used for offline processing. To use FicTrac within a closed-loop setup (to provide real-time feedback for stimuli), you should configure FicTrac to output data via a socket (IP address/port) in real-time. To do this, just set `out_port` to a valid port number in the config file. There is an example Python script for receiving data via sockets in the `scripts` directory.

**Note:** If you encounter issues trying to generate output videos (i.e. `save_raw` or `save_debug`), you might try changing the default video codec via `vid_codec` - see [config params](doc/params.md) for details. If you receive an error about a missing [H264 library](https://github.com/cisco/openh264/releases), you can download the necessary library (i.e. OpenCV 4.0.1 requires `openh264-1.8.0-win64.dll`) from the above link and place it in the `dll` folder under the FicTrac main directory. You will then need to re-run the appropriate `cmake ..` and `cmake --build` commands for your installation.

## Research

If you use FicTrac as part of your research, please cite the original FicTrac publication:

> RJD Moore, GJ Taylor, AC Paulk, T Pearson, B van Swinderen, MV Srinivasan (2014). *"FicTrac: a visual method for tracking spherical motion and generating fictive animal paths"*, Journal of Neuroscience Methods, Volume 225, 30th March 2014, Pages 106-119. [[J. Neuroscience Methods link]](https://doi.org/10.1016/j.jneumeth.2014.01.010) [[Preprint (pdf) link]](https://www.dropbox.com/s/sw6qcmphk417bgi/2014-Moore_etal-JNM_preprint-FicTrac.pdf?dl=0)

This publication contains technical details on how FicTrac works, performance analysis, results, and other discussion.

## Contribution guidelines

If you have modified the FicTrac source code to fix issues, add functionality, or to better suit your setup, please consider making those additions available to other users!

To do so, just follow the standard [Github fork and pull request workflow](https://gist.github.com/rjdmoore/ed014fba0ee2c7e75060ccd01b726cb8).

## License

See the [LICENSE file](LICENSE.txt) for more info.
