import math
import random

import utils


class CameraSensor(object):
  """
  - Homogeneous cameras
  - Static at location
  - Measures in triangle in front (consider contexts)
  - Projection of triangle to line (all marked locations projected onto position on line)
  - Any two regions marked side by side: shape of human detected
  - For localization: other camera with intersecting effective output region, that also detected shape of human
  """

  def __init__(self, location, orientation, viewing_angle_deg):
    self.location = location
    self.orientation = orientation
    self.viewing_angle_deg = viewing_angle_deg
    self.picture = None

  def get_orientation(self):
    return self.orientation

  def get_location(self):
    return self.location

  def get_effective_measurement_region(self):
    measurement_region = set()
    for loc in self.context_region:
      target_orientation = (loc[0] - self.location[0], loc[1] - self.location[1])
      target_angle_deg = utils.get_angle_deg(target_orientation, self.orientation)
      half_viewing_angle_ceil_deg = math.ceil(self.viewing_angle_deg / 2)
      target_angle_floor_deg = math.floor(target_angle_deg)
      if abs(target_angle_floor_deg) <= abs(half_viewing_angle_ceil_deg):
        measurement_region.add(loc)
    return measurement_region

  def get_picture(self):
    return self.picture

  def set_context_region(self, context_region):
    self.context_region = context_region

  def set_picture(self, context_field):
    # TODO: OS.get_context()
    measurement_region = self.get_effective_measurement_region()
    measurement_region = sorted(
      measurement_region,
      key=lambda x: utils.get_distance(x, self.location),
      reverse=True # Descending order, larger distances first
    )
    measurement_region.remove(self.location)
    pixel_dict = {}
    for measurement_location in measurement_region:
      relative_location = (
        measurement_location[0] - self.location[0],
        measurement_location[1] - self.location[1]
      )
      measurement_angle_deg = round(utils.get_directed_angle_deg(relative_location, self.orientation))
      pixel = context_field[measurement_location]["object"]
      if measurement_angle_deg not in pixel_dict:
        pixel_dict[measurement_angle_deg] = pixel
      elif pixel_dict[measurement_angle_deg] == None:
        pixel_dict[measurement_angle_deg] = pixel
    angles = list(pixel_dict.keys())
    angles_sorted = sorted(angles, reverse=True) # For writing to the picture array in the correct order
    self.picture = []
    for angle in angles_sorted:
      pixel = pixel_dict[angle]
      self.picture.append({"pixel" : pixel, "angle" : angle})
    # Pixel array is mapping of angles to positions
    # All locations with angle ~= viewing_angle/2 are on outer sides
    # Have to determine if above or below (negative angle for below orientation, positive angle for above orientation)
    # Locations are in order of distance from camera location (larger distances first)


def test_camera_picture(camera, context_field, max_x, max_y):
  context_region = set(context_field.keys())
  camera.set_context_region(context_region)
  camera.set_picture(context_field)
  picture = camera.get_picture()
  measurement_region = camera.get_effective_measurement_region()
  print_field = {}
  for l in context_field:
    print_field[l] = { "object" : context_field[l]["object"] }
    if l in measurement_region:
      if print_field[l]["object"] != 'H':
        print_field[l]["object"] = 'O'
  utils.print_field(print_field, "object", max_x, max_y)
  for p in picture:
    if p["pixel"] == None:
      print("___", end=" ")
    else:
      print("HHH", end=" ")
  print()
  for p in picture:
    print("%3.0i" % p["angle"], end=" ")
  print()
