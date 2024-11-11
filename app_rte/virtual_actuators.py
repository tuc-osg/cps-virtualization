import itertools
import copy

import physical_actuators


class Brightness(object):
  """ Virtual actuators. """

  def get_combination_output_region(self, combination, region):
    comb_region = combination[0].get_effective_output_region()
    for actuator in combination[1:]:
      eff_out_region = actuator.get_effective_output_region()
      comb_region = comb_region.union(eff_out_region)
    output_region = comb_region.intersection(region)
    return output_region

  def get_actuators_inputs_outputs(self, combination, output_region):
    comb_outputs_inputs = []
    for actuator in combination:
      act_outputs_inputs = actuator.get_available_outputs_inputs_for_region(output_region)
      # = [ (out0, in0), (out1, in1), ... ]
      act_ins_outs = {
        "actuator" : actuator,
        "outputs_inputs" : act_outputs_inputs
      }
      comb_outputs_inputs.append(act_ins_outs)
    return comb_outputs_inputs

  def get_all_output_combinations(self, act_dicts, act_dicts_idx = 0, output_combination = [], output_combinations = []):
    """
    act_dicts = [
      {"actuator" : a0, "outputs_inputs" : [ {"out" : out0, "in" : in0}, {"out" : out1, "in" : in1), ... ]},
      {"actuator" : a1, "outputs_inputs" : [ {"out" : out0, "in" : in0}, {"out" : out1, "in" : in1), ... ]},
      {"actuator" : a2, "outputs_inputs" : [ {"out" : out0, "in" : in0}, {"out" : out1, "in" : in1), ... ]},
      ...
    ]
    """
    if act_dicts_idx == len(act_dicts):
      output_combination_copy = []
      for output in output_combination:
        output_combination_copy.append({
          "actuator": output["actuator"],
          "input_output": copy.deepcopy(output["input_output"])
        })
      output_combinations.append(output_combination_copy)
      return output_combinations
    elif act_dicts_idx == 0:
      output_combination = [None] * len(act_dicts)
    act_dict = act_dicts[act_dicts_idx]
    for input_output in act_dict["outputs_inputs"]:
      output_combination[act_dicts_idx] = {
        "actuator": act_dict["actuator"],
        "input_output": copy.deepcopy(input_output)
      }
      output_combinations = self.get_all_output_combinations(
        act_dicts,
        act_dicts_idx + 1,
        output_combination,
        output_combinations
      )
    return output_combinations

  def get_available_changes_for_combination(self, combination, output_region):
    """
    - Check all combinations of outputs with associated inputs
    [{ "change" : c, "actuators" : [ { "actuator" : a1, "input" : i1 }, { "actuator" : a2, "input" : i2 }, ... ] }]
    """
    actuators_inputs_outputs = self.get_actuators_inputs_outputs(combination, output_region)
    output_combinations = self.get_all_output_combinations(actuators_inputs_outputs, 0, [], [])
    comb_ins_outs = []
    for output_combination in output_combinations:
      out_in_dict = { "actuators" : [] }
      output_sum = 0.0
      for act_dict in output_combination:
        output_sum += act_dict["input_output"]["output"]
        act = {
          "actuator" : act_dict["actuator"],
          "input" : act_dict["input_output"]["input"]
        }
        out_in_dict["actuators"].append(act)
      out_in_dict["change"] = output_sum
      comb_ins_outs.append(out_in_dict)
    return comb_ins_outs

  def select_actuators(self, region, actuators):
    # Only consider brightness actuators that have overlapping effective output region with given region
    acts = []
    for a in actuators:
      if isinstance(a, physical_actuators.BrightnessActuator):
        effective_output_region = a.get_effective_output_region()
        if effective_output_region.intersection(region):
          acts.append(a)
    return acts

  def get_available_changes(self, region, actuators):
    brightness_actuators = self.select_actuators(region, actuators)
    available_changes = []
    for length in range(1, len(actuators) + 1):
      combinations = list(itertools.combinations(brightness_actuators, length))
      for combination in combinations:
        output_region = self.get_combination_output_region(combination, region)
        if output_region:
          comb_available_changes = self.get_available_changes_for_combination(combination, output_region)
          available_changes += comb_available_changes
    return available_changes
