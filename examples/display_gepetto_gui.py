import curves
from multicontact_api import ContactSequence
import gepetto.corbaserver
import pinocchio as pin
from rospkg import RosPack
import time
import argparse
import subprocess
import atexit
import os

# Define robot model 
robot_package_name = "talos_data"
urdf_name = "talos_reduced"
# Define environment
env_package_name = "hpp_environments"
env_name = "multicontact/ground" # default value, may be defined with argument
scene_name = "world"
# timestep used to display the configurations
DT_DISPLAY = 0.04 # 25 fps


def display_wb(robot, q_t):
  t = q_t.min()
  while t <= q_t.max():
      t_start = time.time()
      robot.display(q_t(t))
      t += DT_DISPLAY
      elapsed = time.time() - t_start
      if elapsed > DT_DISPLAY:
          print("Warning : display not real time ! choose a greater time step for the display.")
      else:
          time.sleep(DT_DISPLAY - elapsed)
  # display last config if the total duration is not a multiple of the dt
  robot.display(q_t(q_t.max()))


if __name__ == '__main__':

  # Get cs_name from the arguments:
  parser = argparse.ArgumentParser(description="Load a ContactSequence and display the joint-trajectory in gepetto-gui")
  parser.add_argument('cs_name', type=str, help="The name of the serialized ContactSequence file")
  parser.add_argument('--env_name', type=str, help="The name of environment urdf file in hpp_environments")
  args = parser.parse_args()
  cs_name = args.cs_name
  if args.env_name:
    env_name = args.env_name

  # Start the gepetto-gui background process
  subprocess.run(["killall", "gepetto-gui"])
  process_viewer = subprocess.Popen("gepetto-gui",
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL,
                                      preexec_fn=os.setpgrp)
  atexit.register(process_viewer.kill)

  # Load robot model in pinocchio
  rp = RosPack()
  urdf = rp.get_path(robot_package_name) + '/urdf/' + urdf_name + '.urdf'
  robot = pin.RobotWrapper.BuildFromURDF(urdf, pin.StdVec_StdString(), pin.JointModelFreeFlyer())
  robot.initDisplay(loadModel=True)
  robot.displayCollisions(False)
  robot.displayVisuals(True)

  # Load environment model
  cl = gepetto.corbaserver.Client()
  gui = cl.gui
  env_package_path = rp.get_path(env_package_name)
  env_urdf_path = env_package_path + '/urdf/' + env_name + '.urdf'
  gui.addUrdfObjects(scene_name + "/environments", env_urdf_path, True)


  # Load the motion from the multicontact-api file
  cs = ContactSequence()
  cs.loadFromBinary(cs_name)
  assert cs.haveJointsTrajectories(), "The file loaded do not have joint trajectories stored."
  q_t = cs.concatenateQtrajectories()
  display_wb(robot, q_t)









