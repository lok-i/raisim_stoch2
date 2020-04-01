//
// Created by Jemin Hwangbo on 10/15/27.
// MIT License
//
// Copyright (c) 2027-2027 Robotic Systems Lab, ETH Zurich
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


#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(30);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.06, .6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

int main(int argc, char **argv) {
  /// create raisim world
  raisim::World world;
  
 // world.setGravity({0,0,0}); // by default gravity is set to {0,0,g}
  world.setTimeStep(0.0025);

  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(2600, 1200);
  vis->setImguiSetupCallback(imguiSetupCallback);
  vis->setImguiRenderCallback(imguiRenderCallBack);
  vis->setKeyboardCallback(raisimKeyboardCallback);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);
  vis->setDesiredFPS(25);

  //simulation is automatically stepped, if is false
  raisim::gui::manualStepping = false; 

  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();
  

  /// create visualizer objects
  vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

  /// stoch joint PD controller
  /*jointNominalConfig
   first 3 - base co-ordinates
   next 4 - base orientation
   last 20 - 5 joints per leg*/

  Eigen::VectorXd jointNominalConfig(27), jointVelocityTarget(26);
  Eigen::VectorXd jointState(26), jointForce(26), jointPgain(26), jointDgain(26);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  
  //P and D gains for the leg actuators alone
  jointPgain.tail(20).setConstant(200.0);
  jointDgain.tail(20).setConstant(10.0);

  
  //no of steps per call
  const size_t N = 100;

  auto stoch = world.addArticulatedSystem(raisim::loadResource("Stoch2/Stoch2.urdf"));
  auto stochVis = vis->createGraphicalObject(stoch, "stoch");
  
  stoch->setGeneralizedCoordinate({     0, 0, 0.266, //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        0, 0, 0, 0, 0, //leg 1
                                        0, 0, 0, 0, 0, //leg 2
                                        0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0});

  stoch->setGeneralizedForce(Eigen::VectorXd::Zero(stoch->getDOF()));
  stoch->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  stoch->setPdGains(jointPgain, jointDgain);
  stoch->setName("stoch");
  
  //to take random samples
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.07);
  std::srand(std::time(nullptr));
  stoch->printOutBodyNamesInOrder();
 
  auto wire1 = world.addStiffWire(stoch, 5, {0,0,-0.17}, stoch, 3, {-0.027,0,-0.0325}, 0);
// vis->createGraphicalObject(wire1, "wire1", "red");
  auto wire2 = world.addStiffWire(stoch, 10, {0,0,-0.17}, stoch, 8, {-0.027,0,-0.0325}, 0);
 //vis->createGraphicalObject(wire2, "wire2", "red");
  auto wire3 = world.addStiffWire(stoch, 15, {0,0,-0.17}, stoch, 13, {-0.027,0,-0.0325}, 0);
 //vis->createGraphicalObject(wire3, "wire3", "red");
  auto wire4 = world.addStiffWire(stoch, 20, {0,0,-0.17}, stoch, 18, {-0.027,0,-0.0325}, 0);
 //vis->createGraphicalObject(wire4, "wire4", "red");



  // lambda function for the controller
  auto controller = [&stoch, &generator, &distribution]() {
    static size_t controlDecimation = 0;

    if (controlDecimation++ % 2500 == 0)
      stoch->setGeneralizedCoordinate({0, 0, 0.266,

                                        1, 0, 0,0, 

                                        0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0});

    if (controlDecimation % 50 != 0)
      return;

    /// laikago joint PD controller
    Eigen::VectorXd jointNominalConfig(27), jointVelocityTarget(26);
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0,0,0,0;
  

    float angle = 0;
    bool angle_change = true;
    size_t index = 0;
   // for (size_t i = 0; i < N; i++) {
      


      for (size_t j = 0; j < N; j++) {
        jointNominalConfig << 0, 0, 0,
            0, 0, 0, 0,
            
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0;
        
        for (size_t k = 0; k < stoch->getGeneralizedCoordinateDim() ; k++)
        {
        
//           if(k==7 || k== 12 ||k==17 ||k==22) // abd_motor
 //          jointNominalConfig(k) += ;

          //if(k==8 || k== 13 ||k==18 ||k==23) // upper_hip
          jointNominalConfig(k) += distribution(generator);

       //  if(k==9 || k== 14 ||k==19 ||k==24) // lower_hip
         // jointNominalConfig(k) += distribution(generator) ;

       //  if(k==10 || k== 15 ||k==20 ||k==25) // upper_knee
         //  jointNominalConfig(k) += distribution(generator) ;

         //if(k==11 || k== 16 ||k==21 ||k==26) //lower_knee
          // jointNominalConfig(k) += distribution(generator) ;

         }
       
         //std::cout<<stoch->getGeneralizedCoordinateDim();
      
        stoch->setPdTarget(jointNominalConfig, jointVelocityTarget);
    
      }
    

    //}



  };

  vis->setControlCallback(controller);

  /// set camera
  vis->select(stochVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0.), Ogre::Radian(-1.), 3);

  /// run the app
  vis->run();

  /// terminate
  vis->closeApp();

  return 0;
}
