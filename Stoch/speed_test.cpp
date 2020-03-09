 
//
// Created by jemin on 3/9/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

//
// Created by jhwangbo on 04/09/17.
//

#include "raisim/World.hpp"
#include "/home/lok/Raisim/raisim_workspace/raisimLib/examples/include/benchmarkCommon.hpp"
#include "/home/lok/Raisim/raisim_workspace/raisimLib/examples/include/helper.hpp"
#include <chrono>

int main () {
  raisim::World sim;

  sim.setERP(0.,0.);

  auto checkerBoard = sim.addGround();


// as path for the function below add the path to your urdf file
  auto anymal = sim.addArticulatedSystem("/home/lok/Raisim/raisim_workspace/raisimLib/examples/src/benchmark/Stoch/mesh_obj/stoch_two_obj.urdf");

  struct timespec start, end;
  const int loopN = 100;
  clock_gettime(CLOCK_MONOTONIC, &start);
  for(int i=0; i<loopN; i++)
    sim.integrate();

  clock_gettime(CLOCK_MONOTONIC, &end);
  raisim::print_timediff("anymal", loopN, start, end);
}
