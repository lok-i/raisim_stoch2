# raisim_stoch2
codebase transfer of stoch from pybullet to raisim

## Overview:
This project is an attempt towards shifting the codebase of Stoch2 from the existing pybullet platform to raisim , a simulation engine developed but ETH Zurich owing to its speed and accuracy.

## Experiments:
We tried compairing the speed of simulation of both the example files of ANYmal aswell as our model of Stoch2.

## Inference:
* Raisim proves to be faster as promised when the files of ANYmal was used ,where the 3d files were of     .dae format.
Platform | Speed / frequency (Hz)
------------ | -------------
Raisim | ~23,000
Pybullet | ~15,000

* However PyBullet is magnitudes faster when the files of our Stoch2 bot was used ,where the 3d files were of .obj format
Platform | Speed / frequency (Hz)
------------ | -------------
Raisim | ~6.17
Pybullet | ~5,000

## Ongoing_work:
* Trying to resolve the above discrepancy and validate the accuracy and speed of raisim.
