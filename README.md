# raisim_stoch2
codebase transfer of stoch from pybullet to raisim

## Overview:
This project is an attempt towards shifting the codebase of [Stoch2](https://cps.iisc.ac.in/research/walking-robot/) from the existing pybullet platform to raisim , a simulation engine developed but ETH Zurich owing to its speed and accuracy.

## Experiments:
We tried compairing the speed of simulation of both the example files of ANYmal aswell as our model of Stoch2.

## Inference:
* Raisim proves to be faster as promised when the files of ANYmal was used ,where the 3d files were of     .dae format.

Platform | Speed / frequency (Hz)
------------ | -------------
Raisim | ~23,000
Pybullet | ~15,000

## Work done:
* Our robot platform "Stoch2" was successfully brought in after converting using .obj files for modelling our robot.

## Ongoing work:
* Code base transfer of the bot, from the a low level walking controller all the way towards a GYM environment entirely based on raisim for experimentation.

## General Instructions:
* The user is expected to give the right path's to the resources ie the urdf and the mesh files in their local systems.
* The "Stoch2_renderer" requires the header files of anymal, and hence it can be run and built along with the examples of raisimOgre / a copy of it is to be present in a include folder in your current project directory

