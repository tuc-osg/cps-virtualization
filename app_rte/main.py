"""
- Physical phenomena within factory:
  (- Robot (1x1 field)
    - Location
    - Random motion by direction (up, down, left, right) - consider contexts)

- Light actuators:
  - Static at location
  - Sets brightness in "circle" region to X
"""

import random
import math

import utils
import physical_sensors
import physical_actuators
import virtual_sensors
import human
import rte
import simulation


class NotHuman(object):

  def state():
    return {
      "Shape" : lambda s: s != 'H'
    }

  def priority():
    return 0

  def behavior(_, changes):
    if "Brightness" not in changes:
      return {}
    brightness_changes = [c for c in changes["Brightness"]]
    if brightness_changes == []:
      return {}
    min_change = brightness_changes[0]
    for bc in brightness_changes[1:]:
      if bc["change"] < min_change["change"]:
        min_change = bc
    return { "Brightness" : min_change }


class FactoryLighting(object):
  """ Phenomenon description. """

  def state():
    return {
      "Shape" : lambda s: s == 'H',
    }

  def priority():
    return 1

  def behavior(state, changes):
    if "Brightness" not in changes:
      return {}
    brightness_changes = [c for c in changes["Brightness"] if c["change"] >= 200.0]
    if brightness_changes == []:
      return {}
    min_change = brightness_changes[0]
    for bc in brightness_changes[1:]:
      if bc["actuator_count"] < min_change["actuator_count"]:
        min_change = bc
    return { "Brightness" : min_change }


def main():
  max_x = 30
  max_y = 70
  system_region = utils.create_region(0, max_x, 0, max_y)
  context_regions = [
    utils.create_region(5, 25, 5, 30),
    utils.create_region(5, 25, 35, 65)
  ]
  lights = []
  for x in range(8, max_x, 10):
    for y in range(8, max_y, 10):
      for context_region in context_regions:
        if (x, y) in context_region:
          light_actuator = physical_actuators.BrightnessActuator(location = (x, y), radius = 7, max_lumen = 600.0)
          lights.append(light_actuator)
  viewing_angle = 90
  cameras = []
  # Context 1
  cameras += ([
    physical_sensors.CameraSensor(location = (5, 5), orientation = (0.5, 1), viewing_angle_deg = viewing_angle),
    physical_sensors.CameraSensor(location = (5, 29), orientation = (1, -0.5), viewing_angle_deg = viewing_angle),
    physical_sensors.CameraSensor(location = (24, 29), orientation = (-0.5, -1), viewing_angle_deg = viewing_angle),
    physical_sensors.CameraSensor(location = (24, 5), orientation = (-1, 0.5), viewing_angle_deg = viewing_angle),
  ])
  # Context 2
  cameras += ([
    physical_sensors.CameraSensor(location = (5, 35), orientation = (0.5, 1), viewing_angle_deg = viewing_angle),
    physical_sensors.CameraSensor(location = (5, 64), orientation = (1, -0.5), viewing_angle_deg = viewing_angle),
    physical_sensors.CameraSensor(location = (24, 64), orientation = (-0.5, -1), viewing_angle_deg = viewing_angle),
    physical_sensors.CameraSensor(location = (24, 35), orientation = (-1, 0.5), viewing_angle_deg = viewing_angle),
  ])
  # Context 1
  # cameras += ([
  #   physical_sensors.CameraSensor(location = (5, 5), orientation = (1, 1), viewing_angle_deg = viewing_angle),
  #   physical_sensors.CameraSensor(location = (5, 29), orientation = (1, -1), viewing_angle_deg = viewing_angle),
  #   physical_sensors.CameraSensor(location = (24, 29), orientation = (-1, -1), viewing_angle_deg = viewing_angle),
  #   physical_sensors.CameraSensor(location = (24, 5), orientation = (-1, 1), viewing_angle_deg = viewing_angle),
  # ])
  # # Context 2
  # cameras += ([
  #   physical_sensors.CameraSensor(location = (5, 35), orientation = (1, 1), viewing_angle_deg = viewing_angle),
  #   physical_sensors.CameraSensor(location = (5, 64), orientation = (1, -1), viewing_angle_deg = viewing_angle),
  #   physical_sensors.CameraSensor(location = (24, 64), orientation = (-1, -1), viewing_angle_deg = viewing_angle),
  #   physical_sensors.CameraSensor(location = (24, 35), orientation = (-1, 1), viewing_angle_deg = viewing_angle),
  # ])
  runtime_environment = rte.RuntimeEnvironment([FactoryLighting, NotHuman])
  sim = simulation.FactorySimulation(system_region, max_x, max_y, context_regions, runtime_environment, lights, cameras)
  sim.run()


if __name__ == "__main__":
  main()
