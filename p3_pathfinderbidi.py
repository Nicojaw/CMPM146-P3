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
    front_visit = []
    back_visit = []
    
    front_dist = {}
    back_dist = {}
    
    front_prev = {}
    back_prev = {}
    
    detail_points = {} 
    frontier = []
    
    detail_points[start] = source_point
    detail_points[goal] = destination_point
    
    front_dist[start] = 0
    front_prev[start] = None
    
    back_dist[goal] = 0
    back_prev[goal] = None    
   
    heappush(frontier, (front_dist[start], start, "Goal"))
    heappush(frontier, (back_dist[goal], goal, "Start"))

    while len(frontier) > 0:
      priority, current, pos = heappop(frontier)
      
      if pos == "Start" and current in back_visit:
        break
      elif pos == "Goal" and current in front_visit:
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
        
        nextNeighbor = (min(tmpx2,max(tmpx1,CposX)), min(tmpy2,max(tmpy1,CposY)))
        detail_points[neighbor] = nextNeighbor          
        
        if pos == "Goal":
          new_cost = front_dist[current] + coordinate_distance(detail_points[current], detail_points[neighbor])
        
          if neighbor not in front_dist or new_cost < front_dist[neighbor]:
            visited_nodes.append(neighbor)
            front_visit.append(neighbor)
            front_dist[neighbor] = new_cost 
            front_prev[neighbor] = current
            heappush(frontier, (new_cost + coordinate_distance(detail_points[neighbor], destination_point), neighbor, pos))
        else:
          new_cost = back_dist[current] + coordinate_distance(detail_points[current], detail_points[neighbor])
        
          if neighbor not in back_dist or new_cost < back_dist[neighbor]:
            visited_nodes.append(neighbor)
            back_visit.append(neighbor)
            back_dist[neighbor] = new_cost 
            heappush(frontier, (new_cost + coordinate_distance(detail_points[neighbor], destination_point), neighbor, pos))
            back_prev[neighbor] = current
                        
    if current in front_visit or back_visit:
      backCur = current
      frontCur = current
      pathL = []
      while frontCur:
        prev = front_prev.get(frontCur, None)
        if prev is not None:
          pathL.append((detail_points[frontCur], detail_points[prev]))
        frontCur = prev
      while backCur:
        prev = back_prev.get(backCur, None)
        if prev is not None:
          pathL.append((detail_points[backCur], detail_points[prev]))
        backCur = prev
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