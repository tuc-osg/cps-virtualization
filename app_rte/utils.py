import math

class VectorsDifferentLengthException(Exception): pass


def get_region_from_function(fn, x_min, x_max, y_min, y_max):
  region = set()
  for x in range(x_min, x_max):
    for y in range(y_min, y_max):
      if fn(x, y) == True:
        region.add((x,y))
  return region


def create_region(min_x, max_x, min_y, max_y):
  x_range = (min_x, max_x)
  y_range = (min_y, max_y)
  region = get_region_from_function(
    lambda x, y: x >= x_range[0] and x < x_range[1] and y >= y_range[0] and y < y_range[1],
    x_range[0], x_range[1], y_range[0], y_range[1]
  )
  return region


def get_distance(x1, x2):
  if len(x1) != len(x2):
    raise VectorsDifferentLengthException
  res = 0.0
  for i in range(len(x1)):
    res = res + ((x1[i] - x2[i]) ** 2)
  return math.sqrt(res)


def get_directed_angle_deg(x1, x2):
  directed_angle_rad = math.atan2(x2[1], x2[0]) - math.atan2(x1[1], x1[0])
  return math.degrees(directed_angle_rad)


def get_angle_deg(x1, x2):
  if len(x1) != len(x2):
    raise VectorsDifferentLengthException
  if x1 == (0,0) or x2 == (0,0):
    return 0
  scalar_prod = 0.0
  for i in range(len(x1)):
    scalar_prod += x1[i] * x2[i]
  len_x1 = get_distance(x1, (0,0))
  len_x2 = get_distance(x2, (0,0))
  angle_rad = math.acos(scalar_prod / (len_x1 * len_x2))
  return math.degrees(angle_rad)


def print_region(region, max_x, max_y):
  for x in range(max_x):
    for y in range(max_y):
      if (x, y) in region:
        print("O ", end="")
      else:
        print("- ", end="")
    print()
  print()


def print_field(field, key, max_x, max_y):
  array = [ [' '] * max_y for i in range(max_x) ]
  for location in field:
    if field[location][key] != None:
      array[location[0]][location[1]] = field[location][key]
    else:
      array[location[0]][location[1]] = '-'
  for x in range(max_x):
    for y in range(max_y):
      if isinstance(array[x][y], str):
        print("%3.0c" % array[x][y], end="")
      elif isinstance(array[x][y], float):
        print("{:3.0f}".format(array[x][y]), end="")
    print()
  print()


def get_adjacent_locations(location):
  adj_locations = []
  l = (location[0] + 0, location[1] + 1)
  adj_locations.append(l)
  l = (location[0] + 0, location[1] - 1)
  adj_locations.append(l)
  l = (location[0] + 1, location[1] + 0)
  adj_locations.append(l)
  l = (location[0] + 1, location[1] + 1)
  adj_locations.append(l)
  l = (location[0] + 1, location[1] - 1)
  adj_locations.append(l)
  l = (location[0] - 1, location[1] + 0)
  adj_locations.append(l)
  l = (location[0] - 1, location[1] + 1)
  adj_locations.append(l)
  l = (location[0] - 1, location[1] - 1)
  adj_locations.append(l)
  return adj_locations


def split_disjoint_regions(region):
  separated_regions = []
  while len(region) > 0:
    el = next(iter(region)) # Gets first element from set
    separated_regions.append({el})
    region.remove(el)
    change = True
    while change == True and len(region) > 0:
      change = False
      add_del_locations = []
      for location in separated_regions[-1]:
        adjacent_locations = get_adjacent_locations(location)
        for adj_location in adjacent_locations:
          if adj_location in region and adj_location not in add_del_locations:
            add_del_locations.append(adj_location)
            change = True
      for location in add_del_locations:
        separated_regions[-1].add(location)
        region.remove(location)
  return separated_regions