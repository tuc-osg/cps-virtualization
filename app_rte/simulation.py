import utils
import human
import termcolor
import random


class FactorySimulation(object):

  def __init__(self, system_region, max_x, max_y, context_regions, rte, lights = [], cameras = []):
    self.field = {}
    for location in system_region:
      self.field[location] = { "object" : None, "lumen" : 0.0 }
    self.context_regions = context_regions
    self.max_x = max_x
    self.max_y = max_y
    self.rte = rte
    self.lights = lights
    self.cameras = cameras
    self.max_steps = 1000
    self.persons = []

  def print_simulation_color(self, print_field, step):
    #print(chr(27) + "[2J") # Clear terminal
    array = [ [' '] * self.max_y for i in range(self.max_x) ]
    for location in print_field:
      if print_field[location] != None:
        array[location[0]][location[1]] = print_field[location]
    for x in range(self.max_x):
      for y in range(self.max_y):
        printed = False
        for context_region in self.context_regions:
          if (x, y) in context_region:
            if array[x][y]["object"] == 'H':
              #termcolor.cprint("%4.0f" % array[x][y]["lumen"], "black", "on_red", end='')
              termcolor.cprint("%2.0f" % 0.0, "black", "on_black", end='')
            else:
              if array[x][y]["lumen"] == 0.0:
                termcolor.cprint("%2.0f" % 0.0, "dark_grey", "on_dark_grey", end='')
              else:
                termcolor.cprint("%2.0f" % 0.0, "white", "on_white", end='')
            printed = True
        if printed == False:
          print("XX", end='')
      print()

  def show_simulation(self, step):
    print_field = {}
    for l in self.field:
      print_field[l] = {
        "lumen" : self.field[l]["lumen"],
        "object" : self.field[l]["object"]
      }
    self.print_simulation_color(print_field, step)

  def spawn_person(self, location):
    person = human.Human(location)
    person_region = person.get_region()
    for l in person_region:
      self.field[l]["object"] = 'H'
    self.persons.append(person)
    return person

  def remove_person(self, del_person):
    del_person_region = del_person.get_region()
    not_delete_locations = set()
    for location in del_person_region:
      for person in self.persons:
        if person != del_person:
          if location in person.get_region():
            not_delete_locations.add(location)
    del_person_region = del_person_region - not_delete_locations
    for location in del_person_region:
      self.field[location]["object"] = None
    self.persons.remove(del_person)

  def move_person(self, person, location):
    p = human.Human(location)
    p_region = p.get_region()
    for context_region in self.context_regions:
      if p_region.intersection(context_region) == p_region:
        p = self.spawn_person(location)
        self.remove_person(person)
        return p
    return person

  def rand_move_person(self, person):
    direction = random.randint(0, 3)
    location = person.get_location()
    if direction == 0: # UP
      l = (location[0] - 1, location[1])
    elif direction == 1: # DOWN
      l = (location[0] + 1, location[1])
    elif direction == 2: # LEFT
      l = (location[0], location[1] - 1)
    elif direction == 3: # RIGHT
      l = (location[0], location[1] + 1)
    person = self.move_person(person, l)
    return person

  def initialize(self):
    for context_region in self.context_regions:
      for camera in self.cameras:
        if camera.get_location() in context_region:
          camera.set_context_region(context_region)
          print(camera.get_location())
          utils.print_region(camera.get_effective_measurement_region(), self.max_x, self.max_y)
      for light in self.lights:
        if light.get_location() in context_region:
          light.set_context_region(context_region)
    self.rte.set_actuators(self.lights)
    self.rte.set_sensors(self.cameras)

  def run(self):
    # TODO: Change system_field for individual contexts
    step = 0
    system_region = set(self.field.keys())
    self.rte.set_system_region(system_region)
    self.initialize()
    for step in range(self.max_steps):
      location_1 = (random.randint(5, 23), random.randint(5, 28))
      location_2 = (random.randint(5, 23), random.randint(35, 63))
      person_1 = self.spawn_person(location_1)
      person_2 = self.spawn_person(location_2)
      ordered_descriptions = self.rte.get_ordered_descriptions()
      for description in ordered_descriptions:
        for camera in self.cameras:
          camera.set_picture(self.field)
        self.rte.call_behavior(description)
        lumen_fields = []
        for light in self.lights:
          lumen_field = light.get_lighting()
          lumen_fields.append(lumen_field)
        for location in self.field:
          self.field[location]["lumen"] = 0.0
        for lumen_field in lumen_fields:
          for location in lumen_field:
            self.field[location]["lumen"] += lumen_field[location]
      print()
      print()
      utils.print_field(self.field, "lumen", self.max_x, self.max_y)
      self.show_simulation(step)
      self.remove_person(person_1)
      self.remove_person(person_2)

      # old_location = person_1.get_location()
      # if old_location[0] < 15:
      #   new_location = (old_location[0] + 1, old_location[1])
      # else:
      #   new_location = (old_location[0], old_location[1] + 1)
      # person_1 = self.move_person(person_1, new_location)
      # old_location = person_2.get_location()
      # if old_location[1] > 53:
      #   new_location = (old_location[0], old_location[1] - 1)
      # else:
      #   new_location = (old_location[0] + 1, old_location[1])
      # person_2 = self.move_person(person_2, new_location)