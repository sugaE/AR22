
from __future__ import annotations
import math
import cv2
import numpy as np
import random
from scipy.interpolate import splev, splprep
from RRT import RRT


# class Tree:
#   def __init__(self, x, y, cost=0, id=-1, isroot=False):
#     self.pos = [x, y]
#     self.parent: Tree = None
#     self.cost = cost  # distance to root
#     self.id = id  # ==0 if is root

#   def distance2(self, others: Tree):
#     # return def point2exists(self, point1, points):
#     return np.sum((np.array(others.pos) - np.array(self.pos))**2)

#   def __str__(self):
#     return str({'id': self.id, 'pos': self.pos, 'cost': self.cost, 'parent': self.parent.id if self.parent else None})

def Tree(x, y, cost=0):
  return [int(x), int(y), cost]  # , parent


class RRT_star(RRT):
  # WIN_NAME = 'RRT*_'
  # FILE_PREFIX = 'result/'+WIN_NAME

  def __init__(self, mappath='', maxdistance=50, radius=5, max_edges=10000, collision_dis=1, star_search_dis=0):
    # rrt star search distance when add a new node
    self.search_dis = star_search_dis
    # self.nodes = []  # [x,y,cost,parent]:Tree
    # self.edges = dict()  # key child, value parent
    # ----- Call the superclass constructor
    super().__init__(mappath, maxdistance, radius, max_edges, collision_dis)

  def overrides(self):
    self.WIN_NAME = 'RRT*_'
    self.FILE_PREFIX = 'result/'+self.WIN_NAME

  def buildpaths(self):
    return self.rrt_star()

  def cost(self, ind):
    return self.nodes[ind][2]

  # def calc_cost(self, n1, n2):
  #   return self.nodes[n1][0:2]

  # def pos(self, ind):
  #   return self.nodes[ind][0:2]

  # def parent(self, ind):
  #   return self.nodes[ind][3]

  def rrt_star(self) -> int:
    self.nodes.append(Tree(*self.start_point))
    k = 0
    self.found = 0

    while k <= self.max_edges:
      flag = True
      if self.found == 0:
        # check if reach end point
        dist_end = self.point2exists(self.end_point, [self.nodes[-1]])[0]
        if dist_end <= self.threshold_reach**2 and self.checkLineValid(self.nodes[-1], self.end_point, k):
          print('hoohay!')
          self.end_ind = len(self.nodes)-1
          if dist_end > 0:
            self.edge(self.end_ind, self.end_ind+1)
            self.nodes.append(Tree(*self.end_point, self.cost(self.end_ind) + math.sqrt(dist_end)))
            self.end_ind += 1
          self.found += 1

          self.findpath(self.end_ind)
          self.smooth_path(False)
          while flag:
            print('waiting')
            k = cv2.waitKey(0) & 0xFF
            if k == 32:  # space
              print('continuing')
              flag = False
            else:
              print('ending')
              return self.found

      if not flag:
        continue

      x = random.randrange(0, self.w)
      y = random.randrange(0, self.h)
      if not self.checkPointValid(x, y):
        # TODO return nearest valid point
        continue
      cur_node = Tree(x, y)
      dist_cur = self.point2exists(cur_node, self.nodes)
      ind_near = np.argmin(dist_cur)
      dis_near = dist_cur[ind_near]
      # repeat points
      if dis_near <= 1:
        continue
      near_node = self.nodes[ind_near]
      cur_changed = False

      # check max distance
      if dis_near > self.max_dis**2:
        vec = np.array(cur_node)-np.array(near_node)
        vec = vec/np.linalg.norm(vec)*self.max_dis
        cur_node = Tree(*(near_node + np.array(vec).astype(int)))
        if not self.checkPointValid(*cur_node[0:2]):
          # TODO return nearest valid point
          continue
        cur_changed = True

      # *****************
      if cur_changed:
        dist_cur = self.point2exists(cur_node, self.nodes)

      dist_cur = dist_cur**0.5  # real cost

      inds_near_unsort = np.nonzero(dist_cur < self.search_dis)[0]
      if len(inds_near_unsort):
        # inds_near_unsort = np.argpartition(dist_cur, 5)
        # inds_near = [x[0] for x in inds_near_unsort]
        # inds_near = inds_near_unsort[np.argsort(dist_cur[inds_near_unsort] )]
        min_i = inds_near_unsort[np.argmin(dist_cur[inds_near_unsort]+np.array(self.nodes)[inds_near_unsort, 2])]

        # update cost
        if self.checkLineValid(self.nodes[min_i], cur_node, k):
          min_dist = dist_cur[min_i]+self.cost(min_i)
          cur_node[2] = min_dist
          new_ind = self.connect_edge(min_i, 0, cur_node)
          k += 1
        else:
          continue

        # update other edges cost within area
        cost_gaps = min_dist + dist_cur[inds_near_unsort]-np.array(self.nodes)[inds_near_unsort, 2]
        cost_gaps_ind = np.nonzero(cost_gaps < -1)[0]
        for i in cost_gaps_ind:
          x = inds_near_unsort[i]
          # ++++++++++++++++++++

          if x > 0 and self.checkLineValid(self.nodes[x], cur_node, k) and self.findloop(new_ind, x) < 0:
            # ++++++++++++++++++++++
            self.connect_edge(-1, x)
            t = self.update_cost(x, cost_gaps[i])
            self.connect_edge(new_ind, x)
            if t:
              return self.found
      elif self.checkLineValid(near_node, cur_node, k):
        cur_node[2] = dist_cur[ind_near] + self.cost(ind_near)
        self.connect_edge(ind_near, 0, cur_node)
        k += 1
      # *****************
      else:
        #   # TODO return nearest valid point
        pass

    # no valid path found
    return self.found

  def findloop(self, end, start):
    cur = end
    pre = -1
    while cur:
      if cur == start:
        return pre
      else:
        pre = cur
      cur = self.edges.get(cur)
    return -1


  def update_cost(self, ind, gap):
    if gap >= -1 or ind<=0:
      return 0
    self.nodes[ind][2] = max(0, self.nodes[ind][2] + gap)

    if ind == self.end_ind and self.found > 0:
      self.found += 1
      print('yayhay')
      self.findpath(self.end_ind)
      self.smooth_path(False)
      k = cv2.waitKey(0) & 0xFF
      if k == 32:  # space
        print('continuing')
      else:
        print('ending')
        return 1
    t = [k for k, v in self.edges.items() if int(v) == ind]
    for i in t:
      return self.update_cost(i, gap)
    return 0


if __name__ == '__main__':
  RRT_star('maps/map1.png', max_edges=5000, star_search_dis=50, radius=10, maxdistance=100, collision_dis=2)
