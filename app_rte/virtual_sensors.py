import math

import utils
import physical_sensors

# TODO: Add brightness virtual sensor

class Shape(object):
  """ Virtual Sensor. """

  def __init__(self):
    self.processes = [ self.camera_shape_identification ]

  def is_valid_combination(self, camera1, camera2):
    # TODO: Orientation of cameras is non-sense
    #       I have to check whether cameras observe object from different angles
    #       i.e., have to check in other function
    orientation1 = camera1.get_orientation()
    orientation2 = camera2.get_orientation()
    angle_deg = round(utils.get_angle_deg(orientation1, orientation2))
    if angle_deg == 0 or angle_deg == 180:
      return False
    m_region1 = camera1.get_effective_measurement_region()
    m_region2 = camera2.get_effective_measurement_region()
    intersection = m_region1.intersection(m_region2)
    if not intersection:
      return False
    return True

  def get_shape_regions(self, camera, rule):
    camera_location = camera.get_location()
    orientation = camera.get_orientation()
    picture = camera.get_picture()
    measurement_region = camera.get_effective_measurement_region()
    region = set()
    value = None
    angles = []
    for p in picture:
      if rule(p["pixel"]) == True:
        angle = p["angle"]
        angles.append(angle)
        # Just take last value that satisfies rule
        value = p["pixel"]
        pixel_region = set()
        for measurement_location in measurement_region:
          relative_location = (
            measurement_location[0] - camera_location[0],
            measurement_location[1] - camera_location[1]
          )
          camera_orientation = camera.get_orientation()
          if angle == round(utils.get_directed_angle_deg(relative_location, camera_orientation)):
            pixel_region.add(measurement_location)
        region = region.union(pixel_region)
    return (angles, value, region)

  def merge_results(self, results):
    result_value = results[0]["value"]
    result_region = set()
    for r in results:
      result_region = result_region.union(r["region"])
    return { "value" : result_value, "region" : result_region }

  def camera_shape_identification(self, p_sensors, rule):
    """
    - Create region for shape, such that rule is fulfilled (human shape)
    - Each camera having at least two fields of human side by side
    - Find all suitable combinations of two cameras
    - Identify all humans for all camera combinations, i.e.,
      all separated regions with at least two 'H'
    """
    # TODO: Remember different values for different locations??
    # TODO: Only merge if values are similar?
    cameras = [ps for ps in p_sensors if ps.__class__ == physical_sensors.CameraSensor]
    camera_combinations = []
    for i in range(len(cameras)-1):
      for j in range(i+1, len(cameras)):
        if self.is_valid_combination(cameras[i], cameras[j]):
          camera_combinations.append((cameras[i], cameras[j]))
    results = []
    for camera1, camera2 in camera_combinations:
      angles1, value1, region1 = self.get_shape_regions(camera1, rule)
      angles2, value2, region2 = self.get_shape_regions(camera2, rule)
      # TODO: Check if angles/regions are valid for finding shape
      intersection = region1.intersection(region2)
      results.append({ "value" : value1, "region" : intersection })
    if results == []:
      return None
    result = self.merge_results(results)
    return result

  def exec_output_processes(self, sensors, rule):
    process_results = []
    for process in self.processes:
      output = process(sensors, rule)
      if output != None:
        process_results.append(output)
    if process_results == []:
      return None
    result = self.merge_results(process_results)
    return result


def test_shape(cameras, context_field, max_x, max_y):
  shape_identifier = Shape()
  results = shape_identifier.camera_shape_identification(cameras, lambda s: s == 'H')
  print_field = {}
  for l in context_field:
    print_field[l] = { "object" : '-' }
  for r in results:
    for l in context_field:
      if l in r["region"]:
        print_field[l] = { "object" : 'H' }
  utils.print_field(print_field, "object", max_x, max_y)