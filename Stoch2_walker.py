#!/usr/bin/env python
"""stoch example using the visualizer.

This is the same example as provided in [1], but translated into Python and using the `raisimpy` library (which
is a wrapper around `raisimLib` [2] and `raisimOgre` [3]).

References:
    - [1] https:#github.com/leggedrobotics/raisimOgre/blob/master/examples/src/robots/stoch.cpp
    - [2] raisimLib: https:#github.com/leggedrobotics/raisimLib
    - [3] raisimOgre: https:#github.com/leggedrobotics/raisimOgre
"""

__author__ = ["Jemin Hwangbo (C++)", "Brian Delhaisse (Python)"]
__copyright__ = "Copyright (c), 2019 Robotic Systems Lab, ETH Zurich"
__credits__ = ["Robotic Systems Lab, ETH Zurich + Hwangbo (C++ example code)",
               "Brian Delhaisse (Python wrapper + Python example)"]
__license__ = "MIT"

import os
import numpy as np
import raisimpy as raisim


def normalize(array):
    return np.asarray(array) / np.linalg.norm(array)


def setup_callback():
    vis = raisim.OgreVis.get()

    # light
    light = vis.get_light()
    light.set_diffuse_color(1, 1, 1)
    light.set_cast_shadows(True)
    light.set_direction(normalize([-3., -3., -0.5]))
    vis.set_camera_speed(300)

    # load textures
    vis.add_resource_directory(vis.get_resource_dir() + "/material/checkerboard")
    vis.load_material("checkerboard.material")

    # shadow setting
    manager = vis.get_scene_manager()
    manager.set_shadow_technique(raisim.ogre.ShadowTechnique.SHADOWTYPE_TEXTURE_ADDITIVE)
    manager.set_shadow_texture_settings(2048, 3)

    # scale related settings!! Please adapt it depending on your map size
    # beyond this distance, shadow disappears
    manager.set_shadow_far_distance(10)
    # size of contact points and contact forces
    vis.set_contact_visual_object_size(0.06, 0.6)
    # speed of camera motion in freelook mode
    vis.get_camera_man().set_top_speed(5)


if __name__ == '__main__':
    # create raisim world
    world = raisim.World()
    world.set_time_step(0.0025)

    vis = raisim.OgreVis.get()

    # these methods must be called before initApp
    vis.set_world(world)
    vis.set_window_size(1800, 1200)
    vis.set_default_callbacks()
    vis.set_setup_callback(setup_callback)
    vis.set_anti_aliasing(2)
    vis.set_desired_fps(25)
    raisim.gui.manual_stepping = False

    # starts visualizer thread
    vis.init_app()

    # create raisim objects
    ground = world.add_ground()

    #stoch, stoch_visuals = [], []

    # create visualizer objects
    vis.create_graphical_object(ground, dimension=20, name="floor", material="checkerboard_green")

    # stoch joint PD controller
    joint_nominal_config = np.zeros(27)
    joint_velocity_target = np.zeros(26)
    joint_state = np.zeros(26)
    joint_force = np.zeros(26)
    joint_p_gain = np.zeros(26)
    joint_d_gain = np.zeros(26)

    joint_p_gain[-20:] = 200.
    joint_d_gain[-20:] = 10.

    N = 8
    stoch_urdf_path =  "rsc/Stoch2/Stoch2.urdf"
    stoch=world.add_articulated_system(stoch_urdf_path)
    stoch_visuals=vis.create_graphical_object(stoch, name="stoch")
    stoch.set_generalized_coordinates([0, 0, 0.266, #base co ordinates 
                                        1, 0, 0, 0,  #orientation 
                                        0, 0, 0, 0, 0, #leg 1
                                        0, 0, 0, 0, 0, #leg 2
                                        0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0])
    stoch.set_generalized_forces(np.zeros(stoch.get_dof()))
    stoch.set_control_mode(raisim.ControlMode.PD_PLUS_FEEDFORWARD_TORQUE)
    stoch.set_pd_gains(joint_p_gain, joint_d_gain)
    stoch.set_name("stoch")
    stoch.print_body_names_in_order()
     
    wire1 = world.add_stiff_wire(object1=stoch, local_idx1=5, pos_body1=[0,0,-0.17],
                                    object2=stoch, local_idx2=3, pos_body2=[-0.027,0,-0.0325],length=0.01)

   # vis.create_graphical_object(wire1, name="wire1", material="red")

    wire2 = world.add_stiff_wire(object1=stoch, local_idx1=10, pos_body1=[0,0,-0.17],
                                    object2=stoch, local_idx2=8, pos_body2=[-0.027,0,-0.0325],length=0.01)

    #vis.create_graphical_object(wire2, name="wire2", material="red")

    wire3 = world.add_stiff_wire(object1=stoch, local_idx1=15, pos_body1=[0,0,-0.17],
                                    object2=stoch, local_idx2=13, pos_body2=[-0.027,0,-0.0325],length=0.01)

    #vis.create_graphical_object(wire3, name="wire3", material="red")

    wire4 = world.add_stiff_wire(object1=stoch, local_idx1=20, pos_body1=[0,0,-0.17],
                                    object2=stoch, local_idx2=18, pos_body2=[-0.027,0,-0.0325],length=0.01)

    #svis.create_graphical_object(wire4, name="wire4", material="red")



    class Controller:

        def __init__(self):
            self.control_decimation = 0

        def __call__(self):
            self.control_decimation += 1

            if self.control_decimation % 2500 == 0:
                stoch.set_generalized_coordinates([0, 0, 0.266, #base co ordinates 
                                        1, 0, 0, 0,  #orientation 
                                        0, 0, 0, 0, 0, #leg 1
                                        0, 0, 0, 0, 0, #leg 2
                                        0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0])
            if self.control_decimation % 50 != 0:
                return

            # stoch joint PD controller
            joint_nominal_config = np.array([0, 0, 0, 
                0, 0, 0, 0, 
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                ])
            joint_velocity_target = np.zeros(26)

            joint_config = joint_nominal_config + np.random.normal(0., 0.05, [27])
            stoch.set_pd_targets(joint_config, joint_velocity_target)
            print(joint_config)


    vis.set_control_callback(Controller())

    # set camera
    #vis.select(stoch_visuals)
    camera = vis.get_camera_man().get_camera()
    camera.set_position(1, -2 - 1, 1.5 + 1)
    camera.pitch(1.)

    # run the app
    vis.run()

    # terminate
    vis.close_app()
