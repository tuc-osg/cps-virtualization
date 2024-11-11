import virtual_sensors
import virtual_actuators
import utils
import time


class RuntimeEnvironment(object):

  def __init__(self, phenomenon_descriptions):
    # Ascending order for priorities, such that lower priorities are overwritten later
    self.ordered_descriptions = sorted(
      phenomenon_descriptions,
      key=lambda d: d.priority()
    )
    self.observer = Observer()
    self.controller = Controller()

  def get_ordered_descriptions(self):
    return self.ordered_descriptions

  def set_system_region(self, system_region):
    self.system_region = system_region

  def set_sensors(self, sensors):
    self.sensors = sensors

  def set_actuators(self, actuators):
    self.actuators = actuators

  def call_behavior(self, description):
    # TODO: OS.get_sensors()
    # TODO: OS.get_actuators()
    # TODO: OS.get_contexts()
    # TODO: OS.get_system_region()
    if not description.state(): # Default phenomenon
      state_instances = [{ "state" : None, "region" : self.system_region }]
    else:
      state_instances = self.observer.get_phenomenon_state_instances(description, self.sensors)
    for state_instance in state_instances:
      available_changes = self.controller.get_available_changes(state_instance["region"], self.actuators)
      application_changes = {}
      for property_name in available_changes:
        application_changes[property_name] = []
        for change in available_changes[property_name]:
          application_changes[property_name].append({
            "change" : change["change"],
            "actuator_count" : len(change["actuators"])
          })
      chosen_application_changes = description.behavior(state_instance["state"], application_changes)
      changes = {}
      for property_name in chosen_application_changes:
        chosen_change = chosen_application_changes[property_name]
        print(chosen_change)
        for change in available_changes[property_name]:
          if change["change"] == chosen_change["change"]:
            if len(change["actuators"]) == chosen_change["actuator_count"]:
              changes[property_name] = change
      self.controller.set_changes(changes)

  def call_behaviors(self):
    for description in self.ordered_descriptions:
      self.call_behavior(description)


class Controller(object):

  def __init__(self):
    self.virtual_actuator_classes = { "Brightness" : virtual_actuators.Brightness }

  def get_available_changes(self, region, actuators):
    available_changes = {}
    for property_name in self.virtual_actuator_classes:
      v_actuator_class = self.virtual_actuator_classes[property_name]
      v_actuator = v_actuator_class()
      changes = v_actuator.get_available_changes(region, actuators)
      available_changes[property_name] = changes
    return available_changes

  def set_changes(self, changes):
    for property_name in changes:
      for act_dict in changes[property_name]["actuators"]:
        actuator = act_dict["actuator"]
        input_signal = act_dict["input"]
        actuator.set_input_signal(input_signal)


class Observer(object):
  """
  - Manages state descriptions of phenomena
  - For each state description: get observations of variables
  - Intersect regions of all variables
  - Split result into separate regions
  - Call application for each region
  """

  def __init__(self):
    self.virtual_sensor_classes = { "Shape" : virtual_sensors.Shape }

  def get_phenomenon_state_instances(self, description, sensors):
    results = {}
    state_description = description.state()
    for property_name in state_description:
      if property_name not in self.virtual_sensor_classes:
        return []
      rule = state_description[property_name]
      v_sensor_class = self.virtual_sensor_classes[property_name]
      v_sensor = v_sensor_class()
      results[property_name] = v_sensor.exec_output_processes(sensors, rule)
      if results[property_name] == None:
        return []
    instances = []
    property_name_list = list(results.keys())
    result_region = results[property_name_list[0]]["region"]
    for property_name in results:
      property_region = results[property_name]["region"]
      result_region = result_region.intersection(property_region)
    state_instance_regions = utils.split_disjoint_regions(result_region)
    state_instances = []
    # TODO: Assigning same value to all regions does not really make sense,
    #       if different values for different regions are possible (see todo virtual_sensors.py)
    for region in state_instance_regions:
      state = {}
      for property_name in results:
        state[property_name] = results[property_name]["value"]
      state_instance = { "state" : state, "region" : region }
      state_instances.append(state_instance)
    return state_instances


def test_rte(context_field, phenomenon_descriptions, cameras, lights):
  context_region = set(context_field.keys())
  for camera in cameras:
    camera.set_context_region(context_region)
  for light in lights:
    light.set_context_region(context_region)
  rte = RuntimeEnvironment(phenomenon_descriptions)
  rte.set_system_region(context_region)
  rte.set_sensors(cameras)
  rte.set_actuators(lights)
  rte.call_behaviors()