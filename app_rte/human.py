import random


class Human(object):

  def __init__(self, location):
    self.location = location
    self.region = self.get_region_from_location(self.location)

  def get_region_from_location(self, location):
    region = {
      (location[0], location[1]),
      (location[0] + 1, location[1]),
      (location[0], location[1] + 1),
      (location[0] + 1, location[1] + 1)
    }
    return region

  def get_location(self):
    return self.location

  def get_region(self):
    return self.region