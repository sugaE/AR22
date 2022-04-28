
from __future__ import annotations
from operator import truediv
import cv2
import numpy as np
import random
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

def Tree(x, y, cost=0, parent=-1):
  return [x, y, cost]  # , parent


class RRT_star(RRT):
  WIN_NAME = 'RRT*_'
  FILE_PREFIX = 'result/'+WIN_NAME

  def __init__(self, maxdistance=50, radius=5, max_edges=10000, collision_dis=1, star_search_dis=0):
    # rrt star search distance when add a new node
    self.search_dis = star_search_dis
    # self.nodes = []  # [x,y,cost,parent]:Tree
    # self.edges = dict()  # key child, value parent
    # ----- Call the superclass constructor
    super().__init__(maxdistance, radius, max_edges, collision_dis)

  def buildpaths(self):
    self.rrt_star()

  def cost(self, ind):
    return self.nodes[ind][2]

  # def pos(self, ind):
  #   return self.nodes[ind][0:2]

  # def parent(self, ind):
  #   return self.nodes[ind][3]

  def rrt_star(self) -> int:
    self.nodes.append(Tree(*self.start_point))
    k = 0

    while k <= self.max_edges:
      # check if reach end point
      dist_end = self.point2exists(self.end_point, [self.nodes[-1]])
      if dist_end <= self.radius**2:
        print('hoohay!')
        self.end_ind = len(self.nodes)-1
        if dist_end > 0:
          self.edge(self.end_ind, self.end_ind+1)
          self.nodes.append(Tree(*self.end_point, self.cost(self.end_ind) + dist_end))
          self.end_ind += 1
        return 1

      x = random.randrange(0, self.w)
      y = random.randrange(0, self.h)
      if not self.checkPointValid(x, y):
        # TODO return nearest valid point
        continue
      cur_node = Tree(x, y)
      dist_cur = self.point2exists(cur_node, self.nodes)
      ind_near = np.argmin(dist_cur)
      dis_near = dist_cur[ind_near]
      near_node = self.nodes[ind_near]
      cur_changed = False
      # repeat points
      if dis_near <= 0:
        continue
      # check max distance
      if dis_near > self.max_dis**2:
        vec = np.array(cur_node)-np.array(near_node)
        vec = vec/np.linalg.norm(vec)*self.max_dis
        cur_node = Tree(*(near_node + np.array(vec).astype(int)))
        cur_changed = True

      # check if exists obsticles along path
      t1 = (near_node[0], cur_node[0]) if near_node[0] <= cur_node[0] else (cur_node[0], near_node[0])
      t2 = (near_node[1], cur_node[1]) if near_node[1] <= cur_node[1] else (cur_node[1], near_node[1])
      partial_map = self.map[t1[0]:t1[1], t2[0]:t2[1]]
      if partial_map.shape[1] == 0 or partial_map.shape[0] == 0:
        # TODO deal with line
        continue

      # fix boundary issues: -1
      line_msk = cv2.line(np.zeros(partial_map.shape), (0, 0), (partial_map.shape[1]-1, partial_map.shape[0]-1), 1, 1)

      # print('[k]', k+1, ':', partial_map, line_msk, partial_map.shape, t1, t2)

      if line_msk.sum() == np.multiply(partial_map, line_msk).sum():
          # self.linkrrt(cur_node, ind_near, dist_cur)
        # add new node and edge
        self.nodes.append(cur_node)
        lst_edge = [ind_near, len(self.nodes)-1]
        # self.edges[lst_edge[1]] = lst_edge[0]
        self.edge(*lst_edge)
        k += 1

        # cv2.circle(self.mapshow, cur_node, 2, (0, 0, 255), -1)
        # cv2.line(self.mapshow, self.nodes[lst_edge[0]], self.nodes[lst_edge[1]], (100, 100, 100))
        # # cv2.putText(self.mapshow, str(k), cur_node, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=.3, color=(111, 111, 111))

        # # Using cv2.putText() method
        # cv2.imshow(RRT.WIN_NAME, self.mapshow)

        # *****************
        # self.linkrrt(cur_node, ind_near, dist_cur)
        if cur_changed:
          dist_cur = self.point2exists(cur_node, self.nodes)
          # dist_cur = dist_cur[dist_cur < self.search_dis]
        inds_near_unsort = np.argwhere(dist_cur < self.search_dis)
        # inds_near_unsort = np.argpartition(dist_cur, 5)
        inds_near = inds_near_unsort[np.argsort(self.nodes[inds_near_unsort], 2)]
        for i in inds_near:
          dist_i = dist_cur[i]

          pass
        # *****************

      else:
        # TODO return nearest valid point
        pass

    # no valid path found
    return 0


if __name__ == '__main__':
  RRT_star()
