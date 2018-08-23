<div align="left">
  <img src="http://rjdmoore.net/fictrac/header_text.jpg"><br><br>
</div>

**FicTrac** is an open source software library and set of executables for reconstructing the fictive path of an animal walking on a patterned sphere. The software is fast, flexible, and easy to use and simplifies the setup of closed-loop tracking experiments.

FicTrac was originally developed by researchers at the [Queensland Brain Institute](http://qbi.uq.edu.au/) at the University of Queensland, Australia for tracking honeybees and fruit flies during closed-loop tethered walking experiments, but it has since proved useful for tracking a wide range of animals with different movement speeds, track ball diameters and patterns, and stimuli.

You'll find everything you need to [get started](#getting-started) with FicTrac in the sections below.

You might also be interested in the following links:
* [Demo video](http://youtu.be/BeGYOEOdWjw) - Quick (30s) overview of what FicTrac does and how it works.
* [FicTrac manual](http://link) - Detailed instructions, description of output data, parameters, recommendations, etc.
* [Homepage](http://fictrac.rjdmoore.net) - Contact details for the main author/developer, links, and further info.
* [Forum](http://www.reddit.com/r/fictrac/) - Subreddit for FicTrac users to share issues and advice.
* [Mailing list](http://fictrac.rjdmoore.net/mail.html) - Subscribe to receive important announcements and updates.
* [Journal paper](http://doi.org/10.1016/j.jneumeth.2014.01.010) - Technical details, analysis, results, etc. You'll also find citation info and a copy of the preprint (pdf) further down this page.

Happy tracking!

## Getting started

If you're just setting up your lab, or wondering whether FicTrac is suitable for your setup (spoiler: yes, probably), check the [section below](#experimental-setup) for the basic requirements.

If you already have an experimental enclosure with a camera, you can use FicTrac to either process those videos offline or to run live from the camera. Just follow the sections below to [install](#installation), [configure](#configuration), and [run FicTrac](#running-fictrac).

### Experimental setup

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

The FicTrac source code can be built for both Windows and Linux operating systems. You can also build and run FicTrac from within a Ubuntu (Linux) virtual machine on any operating system.

#### Windows

1. Download and install required dependencies:
    1. [Cmake build system](https://cmake.org/download/) (binary distribution)
    2. [OpenCV computer vision library](https://opencv.org/releases.html) (latest release Win pack)
    3. [NLopt optimisation library](https://nlopt.readthedocs.io/en/latest/NLopt_on_Windows/) (precompiled DLL)
2. Clone or download the FicTrac repository.
3. Navigate to the FicTrac folder and open a terminal.
4. Create a new folder and change directory:
```
mkdir build
cd build
```
5. Next, we will configure and build the FicTrac project. FicTrac is written in C++, so you'll need a suitable compiler. In this example we will use MSVS Build Tools. If you don't already have Visual Studio, you will need to install the [build tools](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2017).
6. Run Cmake to prepare the necessary build files for FicTrac. Here we also need to provide the paths to where we installed OpenCV and NLopt (I have given example paths here, you will need to modify them for your installation).
```
cmake -G "Visual Studio 15 2017 Win64" -D OPENCV_DIR=C:\path\to\opencv-3.4.2\build -D NLOPT_DIR="C:\path\to\nlopt-2.4.2\" ..
```
7. Finally, build and install FicTrac:
```
cmake --build . --config Release --target ALL_BUILD
```

If everything went well, the executables for FicTrac and the configuration utility will be placed in the `bin` directory under the FicTrac source folder.

#### Ubuntu (Linux)




Blah ..

* [Building FicTrac for Windows](http://link)
* [Building FicTrac for Ubuntu](http://link)

* [Ubuntu within VM on Windows OS](http://link)
* [Pre-built binaries](http://link)

#### Ubuntu (Linux) virtual machine



### Configuration

Blah ..

* [Configuration guide](http://link)

### Running FicTrac

Blah ..

* [More detailed user guide](http://link)

#### Windows

```shell
$ blah
```

#### Ubuntu

```shell
$ blah
```

## Research

citing ...

## Contribution guidelines

Policy ...

## For more information
* [Link 1](https://link)
* [Link 2](https://link)
* Bugs, etc

## License

[CC BY-NC-SA 3.0](LICENSE.txt)
