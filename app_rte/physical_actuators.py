import utils


class BrightnessActuator(object):

  def __init__(self, location, radius, max_lumen):
    self.location = location
    self.radius = radius
    self.max_lumen = max_lumen
    self.set_input_signal(0) # Initialize light to be off

  def get_location(self):
    return self.location

  def get_effective_output_region(self):
    output_region = set()
    for location in self.context_region:
      distance = utils.get_distance(location, self.location)
      if distance <= self.radius:
        output_region.add(location)
    return output_region

  def set_input_signal(self, input_signal):
    if input_signal != 0 and input_signal != 1:
      print("Wrong input signal!")
      return # TODO: Throw exception?
    self.input_signal = input_signal

  def get_location_lumen(self, location):
    if location == self.location:
      return self.max_lumen
    distance = utils.get_distance(location, self.location)
    brightness = self.max_lumen / (distance ** 0.8)
    return brightness

  def set_context_region(self, context_region):
    self.context_region = context_region

  def get_lighting(self):
    # TODO: OS.get_context()
    output_region = self.get_effective_output_region()
    output_field = {}
    for location in output_region:
      if self.input_signal == 0:
        output_field[location] = 0.0
      elif self.input_signal == 1:
        output_field[location] = self.get_location_lumen(location)
    return output_field

  def get_available_outputs_inputs_for_region(self, region):
    output_region = self.get_effective_output_region()
    influence_region = output_region.intersection(region)
    if not influence_region:
      return []
    sum_lumen = 0.0
    for location in influence_region:
      sum_lumen += self.get_location_lumen(location)
    avg_lumen = sum_lumen / len(influence_region)
    available_outputs_inputs = []
    if self.input_signal == 0: # Increase Lumen by turning light on
      available_outputs_inputs.append({ "output" : avg_lumen, "input" : 1 })
      # available_outputs_inputs.append({ "output" : 0.0, "input" : 0 })
      available_outputs_inputs.append({ "output" : -avg_lumen, "input" : 0 })
    else: # Decrease Lumen by turning light off
      available_outputs_inputs.append({ "output" : -avg_lumen, "input" : 0 })
      # available_outputs_inputs.append({ "output" : 0.0, "input" : 1 })
      available_outputs_inputs.append({ "output" : avg_lumen, "input" : 1 })
    return available_outputs_inputs


def test_brightness_actuator(light_actuator, input_signal, context_field, max_x, max_y):
  context_region = set(context_field.keys())
  light_actuator.set_context_region(context_region)
  light_actuator.set_input_signal(input_signal)
  context_field = light_actuator.get_lighting(context_field)
  print_field = {}
  for l in context_field:
    print_field[l] = { "lumen" : context_field[l]["lumen"] }
  utils.print_field(print_field, "lumen", max_x, max_y)

def test_brightness_actuators(light_actuators, input_signal, context_field, max_x, max_y):
  context_region = set(context_field.keys())
  for light in light_actuators:
    light.set_context_region(context_region)
    light.set_input_signal(input_signal)
    context_field = light.get_lighting(context_field)
  print_field = {}
  for l in context_field:
    print_field[l] = { "lumen" : context_field[l]["lumen"] }
  utils.print_field(print_field, "lumen", max_x, max_y)