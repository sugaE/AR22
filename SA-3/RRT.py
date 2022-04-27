

import imp
from math import ceil
import cv2
from PIL import Image
import numpy as np
from scipy.spatial import distance_matrix
import random
import json


class RRT:
  WIN_NAME = 'World'
  DEBUG = False

  def __init__(self, maxdistance=50, radius=5, max_edges=10000):
    # self.mapshow
    self.start_point = None
    self.end_point = None
    self.end_ind = None
    self.max_dis = maxdistance
    self.radius = radius
    self.max_edges = max_edges
    self.nodes = []
    self.edges = []
    self.loadmap()
    self.path = []
    if RRT.DEBUG:
      self.mapshow = cv2.imread('mapshow.png')
      cv2.imshow(RRT.WIN_NAME, self.mapshow)
      f = open('data.json')
      data = json.load(f)
      self.start_point = data['start_point']
      self.end_ind = data['end_ind']
      self.nodes = data['nodes']
      self.edges = data['edges']
      f.close()
    else:
      path_exist = self.rrt()
      if path_exist:
        cv2.imwrite('mapshow.png', self.mapshow)
        with open('data.json', 'w') as f:
          json.dump({'start_point': self.start_point, 'end_ind': int(self.end_ind), 'nodes': np.array(self.nodes).tolist(), 'edges': np.array(self.edges).tolist()}, f)
    self.findpath()

  def loadmap(self):
    # read map
    self.maporigin = cv2.imread('maps/map1.png')
    # inflate map
    self.mapshow = cv2.erode(self.maporigin, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.radius*2, self.radius*2)))
    # self.map = np.array(self.mapshow).T // 255
    self.map = (np.array(cv2.cvtColor(self.mapshow, cv2.COLOR_RGB2GRAY)).T)//255
    self.mapshow = ((255-self.mapshow)//2 & self.maporigin) + self.maporigin

    self.w, self.h = self.map.shape
    print(self.w, self.h)

    cv2.namedWindow(RRT.WIN_NAME)
    if RRT.DEBUG:
      return

    cv2.imshow(RRT.WIN_NAME, self.mapshow)
    self.flag_readpos = 2
    cv2.setMouseCallback(RRT.WIN_NAME, self.readPos)

    while self.flag_readpos:
      cv2.waitKey(1)

    print(self.start_point, self.end_point)

  def checkPointValid(self, x, y):
      return self.map[x, y] > 0

  def readPos(self, event, x, y, flags, param):
    if not self.flag_readpos:
      return

    isValid = self.checkPointValid(x, y)
    if event == cv2.EVENT_MOUSEMOVE:
      mapCopy = self.mapshow.copy()
      cv2.circle(mapCopy, (x, y), self.radius, (0, 0, 255) if not isValid else ((0, 255, 0)if self.flag_readpos == 2 else (255, 0, 0)), -1)
      cv2.imshow(RRT.WIN_NAME, mapCopy)
      # check if in obsticle
    if event == cv2.EVENT_LBUTTONDOWN and isValid:
      self.flag_readpos = max(self.flag_readpos-1, 0)
      if self.flag_readpos == 1:
        self.start_point = [x, y]
        cv2.circle(self.mapshow, (x, y), self.radius, (0, 255, 0), -1)
      else:
        self.end_point = [x, y]
        cv2.circle(self.mapshow, (x, y), self.radius, (255, 0, 0), -1)
      cv2.imshow(RRT.WIN_NAME, self.mapshow)

    # Algorithm BuildRRT
    #     Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
    #     Output: RRT graph G

    #     G.init(qinit)
    #     for k = 1 to K do
    #         qrand ← RAND_CONF()
    #         qnear ← NEAREST_VERTEX(qrand, G)
    #         qnew ← NEW_CONF(qnear, qrand, Δq)
    #         G.add_vertex(qnew)
    #         G.add_edge(qnear, qnew)
    #     return G

  def point2exists(self, point):
    return np.sum((np.array(self.nodes) - np.array(point))**2, axis=1)

  def rrt(self):
    self.nodes.append(self.start_point)
    k = 0

    while k <= self.max_edges:
      # check if reach end point
      dist_end = self.point2exists(self.end_point)
      if np.min(dist_end) <= self.radius**2:
        # draw final path
        print('hoohay!')
        self.end_ind = np.argmin(dist_end)
        return 1

      x = random.randrange(0, self.w)
      y = random.randrange(0, self.h)
      if not self.checkPointValid(x, y):
        # return nearest valid point
        continue
      cur_node = [x, y]
      dist_cur = self.point2exists(cur_node)
      ind_near = np.argmin(dist_cur)
      dis_near = dist_cur[ind_near]
      near_node = self.nodes[ind_near]
      # check max distance / repeat points
      if dis_near > 0 and dis_near > self.max_dis**2:
        vec = np.array(cur_node)-np.array(near_node)
        vec = vec/np.linalg.norm(vec)*self.max_dis
        cur_node = near_node + np.array(vec).astype(int)

      # check if exists obsticles along path
      t1 = (near_node[0], cur_node[0]) if near_node[0] <= cur_node[0] else (cur_node[0], near_node[0])
      t2 = (near_node[1], cur_node[1]) if near_node[1] <= cur_node[1] else (cur_node[1], near_node[1])
      partial_map = self.map[t1[0]:t1[1], t2[0]:t2[1]]
      if partial_map.shape[1] == 0 or partial_map.shape[0] == 0:
        # deal with line
        continue

      # boundary issues: -1
      line_msk = cv2.line(np.zeros(partial_map.shape), (0, 0), (partial_map.shape[1]-1, partial_map.shape[0]-1), 1, 1)

      # print('[k]', k+1, ':', partial_map, line_msk, partial_map.shape, t1, t2)

      if line_msk.sum() == np.multiply(partial_map, line_msk).sum():
        # add new node and edge
        self.nodes.append(cur_node)
        lst_edge = [ind_near, len(self.nodes)-1]
        self.edges.append(lst_edge)
        k += 1

        cv2.circle(self.mapshow, cur_node, 2, (0, 0, 255), -1)
        cv2.line(self.mapshow, self.nodes[lst_edge[0]], self.nodes[lst_edge[1]], (100, 100, 100))
        # cv2.putText(self.mapshow, str(k), cur_node, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=.3, color=(111, 111, 111))

        # Using cv2.putText() method
        cv2.imshow(RRT.WIN_NAME, self.mapshow)
      else:
        # return nearest valid point
        pass

    # no valid path found
    return 0

  def findpath(self):
    cur = self.end_ind
    self.path.append(cur)
    # while cur != 0:
    #   # near_node = self.nodes[cur]
    #   self.edges.index(lambda x: x)

    # route = np.asarray(self.path)
    # print(route.shape)

    for i in range(len(self.edges), 0, -1):
      edge = self.edges[i-1]
      if edge[1] == cur:
        cur = edge[0]
        self.path.append(cur)
        cv2.line(self.mapshow, self.nodes[edge[0]], self.nodes[edge[1]], (0, 200, 200), round(self.radius/2))
        if cur != 0:
          cv2.circle(self.mapshow, self.nodes[cur], self.radius, (200, 200, 0), -1)

    cv2.imshow(RRT.WIN_NAME, self.mapshow)
    self.path.reverse()
    print(self.path)

    while 1:
      k = cv2.waitKey(1) & 0xFF
      if k == 27:
          break
    cv2.destroyAllWindows()


if __name__ == '__main__':
  RRT(max_edges=10000)
