# raisim_stoch2
codebase transfer of stoch from pybullet to raisim

## Overview:
This project is an attempt towards shifting the codebase of [Stoch2](https://cps.iisc.ac.in/research/walking-robot/) from the existing pybullet platform to raisim , a simulation engine developed by ETH Zurich owing to its speed.Taking a learning based approach which is purely data driven,certain aspects of the simulation platform hugely affect the feasibility and transferability of the very solution that is being developed.One of the key elements that makes Raisim a viable alternative is its speed,as it could speed up the process of training by a great magnitude.Also, the direct deployment of policies learnt by an agent in this platform into a real world robot ANYmal looks quite appealing.   

## Experiments:
We tried compairing the speed of simulation with our robot model Stoch incorporated in the platform.We conducted seperates test from with implementations in both C++ and python whose results are tabulated below.

## Inference:
* Raisim proves to be faster as promised and the collision bodies look far more accurate.
* Though Raisim is faster,upon usage of [raisimpy](https://github.com/robotlearn/raisimpy) - a third party python wrapper for raisim the speed felldown by a considerable amount but still is faster than our previous bullet engine.

Tested with Stoch2:-

Platform | Speed / frequency
------------ | -------------
Raisim(in C++) |~220kHz 
bullet engine | ~2637 Hz

Tested with ANYmal:-

Platform | Speed / frequency (Hz)
------------ | -------------
Raisim(in python) |~110kHz 
Raisim(in C++) | ~86 kHz

Thus,as compared to our previous bullet engine Raisim proves to be **83** times faster.The drop upon usage of raisimpy is a minor tradeoff given the fast prototyping that we get in python.A more quantative study has been done by the very developers of the platform([SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/)). 


## Ongoing work:
* Development of a OpenAI Gym based environment for conducting our tests and speed up our experimentaion
* Trying out different approaches to account for the lower speed upon the usage of raisimpy.

## General Instructions:
* The repo could be directly downloaded and built using cmake if the required libraries are installed.([raisimlib](https://github.com/leggedrobotics/raisimLib),[raisimOgre](https://github.com/leggedrobotics/raisimOgre))
* The python implementations could be directly run by python if the module [raisimpy](https://github.com/robotlearn/raisimpy) is installed.

