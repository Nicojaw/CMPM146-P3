# Nico Williams
# nijowill
# CMPM146 P3 - NavMesh Pathfinding


# For the sake of complete acacdemic honesty, 
# The code I am basing my pathing algorithms off of is from p1 code
# that I did with Samuel Reha (sreha). That directory can be found here 
# https://github.com/SamReha/CMPM146-P1

from heapq import heappush, heappop
from math import sqrt

def find_path(source_point, destination_point, mesh):
  path = []
  visited_nodes = []
  src = None
  dst = None
  
  def dijk(start, goal, graph):
    frontier = []
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    detail_points = {} 
    detail_points[start] = source_point
    detail_points[goal] = destination_point
   
    heappush(frontier, (cost_so_far[start], start))

    while len(frontier) > 0:
      _, current = heappop(frontier)
      
      if current == goal:
        break

      neighborhood = graph['adj'].get(current, [])

      for neighbor in neighborhood:
        # Find the current position of the point in the box, record the detail points for line drawings
        # Find the distance of each neighbor, go to the closest one. 
        Cpos = detail_points[current]
        CposX = Cpos[0]
        CposY = Cpos[1]
            
        tmpx1 = neighbor[0]
        tmpx2 = neighbor[1]
        tmpy1 = neighbor[2]
        tmpy2 = neighbor[3]
        
        nextNeighbor = (min(tmpx2,max(tmpx1 + 10,CposX)), min(tmpy2 + 10,max(tmpy1,CposY)))
        detail_points[neighbor] = nextNeighbor          
            
        new_cost = cost_so_far[current] + coordinate_distance(detail_points[current], detail_points[neighbor])
        
        if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
          visited_nodes.append(neighbor)
          cost_so_far[neighbor] = new_cost 
          heappush(frontier, (new_cost + coordinate_distance(detail_points[neighbor],destination_point),neighbor))
          came_from[neighbor] = current
          
    if current == goal:
      pathL = []
      detail_points[start] = source_point
      detail_points[goal] = destination_point
      while current:
        if came_from[current] is not None:
          pathL.append((detail_points[current], detail_points[came_from[current]]))
        current = came_from[current]
      return pathL
    else:
      return []	
 
 
# A helper function to compute the distance between to coordinates. 
# Taken from my dijkstra.py from p1. 
  def coordinate_distance(coord1, coord2):
	x1 = coord1[0]
	x2 = coord2[0]

	y1 = coord1[1]
	y2 = coord2[1]

	return sqrt((x1-x2)**2+(y1-y2)**2)
 
  
# A helper function to tell if a point is in a box.
  def findBox (point, box):
    x = point[0]
    y = point[1]
    bx1 = box[0]
    bx2 = box[1]
    by1 = box[2]
    by2 = box[3]
    
    if x > bx1 and x <= bx2 and y > by1 and y <= by2:
      return True
    else: 
      return False
 
# Find the sorce and destination boxes.  
  for box in mesh['boxes']:
    if findBox(source_point, box):
      src = box
    if findBox(destination_point, box):
      dst = box
      
  if src == dst:
    path = [(source_point, destination_point)]
    visited_nodes = [src]
  else:  
    path = dijk(src, dst, mesh) 
    #print path
    
  if len(path) == 0:
    print "No Path!"
  return (path, visited_nodes)