'''
RRT:
https://www.youtube.com/watch?v=QR3U1dgc5RE

B-Spine:
https://www.researchgate.net/publication/224264221_Interactive_Locomotion_Animation_using_Path_Planning
https://opensourc.es/blog/b-spline/
https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.BSpline.html#scipy.interpolate.BSpline
'''
import cv2
import numpy as np
import random
import json
from scipy.interpolate import splev, splprep


def Tree(x, y, cost=0):
  return [int(x), int(y)]  # , parent

class RRT:
  DEBUG = False

  def __init__(self, mappath='', maxdistance=50, radius=5, max_edges=10000, collision_dis=1):

    self.WIN_NAME = 'RRT_'
    self.FILE_PREFIX = 'result/'+self.WIN_NAME

    # self.mapshow
    self.start_point = None
    self.end_point = None
    self.end_ind = None
    self.max_dis = maxdistance
    self.radius = radius
    self.threshold_reach = self.radius*2
    self.max_edges = max_edges
    self.collision_dis = collision_dis
    self.mappath = mappath
    self.nodes = []
    self.edges = {}
    self.path = []
    self.found = 0
    self.overrides()
    self.run()

  def overrides(self):
    pass

  def run(self):
    self.loadmap()
    path_exist = False
    if RRT.DEBUG:
      self.mapshow = cv2.imread(self.FILE_PREFIX+'result.png')
      cv2.imshow(self.WIN_NAME, self.mapshow)
      f = open(self.FILE_PREFIX+'data.json')
      data = json.load(f)
      self.start_point = data['start_point']
      self.end_point = data['end_point']
      self.end_ind = data['end_ind']
      self.nodes = data['nodes']
      self.edges = data['edges']
      path_exist = data['path_exist']
      f.close()
    else:
      path_exist = self.buildpaths()
      self.dump_result(path_exist)
    if path_exist:
      self.findpath(self.end_ind)
      self.smooth_path(False)
    self.blockui()

  def dump_result(self, path_exist=1):
    cv2.imwrite(self.FILE_PREFIX+'result.png', self.mapshow)
    with open(self.FILE_PREFIX+'data.json', 'w') as f:
      json.dump({
          'path_exist': path_exist,
          'start_point': self.start_point,
          'end_point': self.end_point,
          'end_index': int(self.end_ind or -1),
          'nodes': np.array(self.nodes).tolist(),
          'edges': self.edges
      }, f)
      #
    print(path_exist)

  def buildpaths(self):
    return self.rrt()
    #  if self.search_dis <= 0:
    #     path_exist = self.buildpaths()
    #   else:
    #     path_exist = self.rrt_star()

  def loadmap(self):
    #
    # read map
    self.maporigin = cv2.imread(self.mappath or 'maps/map1.png')
    # inflate map
    inflate_dis = self.radius*2+self.collision_dis
    self.mapshow = cv2.erode(self.maporigin, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (inflate_dis, inflate_dis)))
    # self.map = np.array(self.mapshow).T // 255
    self.map = (np.array(cv2.cvtColor(self.mapshow, cv2.COLOR_RGB2GRAY)).T)//255
    self.mapshow = ((255-self.mapshow)//2 & self.maporigin) + self.maporigin
    cv2.imwrite(self.FILE_PREFIX+'mapshowbk.png', self.mapshow)

    self.w, self.h = self.map.shape
    print(self.w, self.h)

    cv2.namedWindow(self.WIN_NAME)
    if RRT.DEBUG:
      return

    cv2.imshow(self.WIN_NAME, self.mapshow)
    self.flag_readpos = 2
    cv2.setMouseCallback(self.WIN_NAME, self.readPos)

    while self.flag_readpos:
      cv2.waitKey(1)

    print(self.start_point, self.end_point)

  def checkPointValid(self, x, y):
    # radius check will be done in line check
    return self.map[x, y] > 0  # and self.map[x-self.radius:x+self.radius, y-self.radius:y+self.radius]

  def checkLineValid(self, p_from, p_to, k=0):
    # check if exists obsticles along path
    t1 = [p_from[0], p_to[0]] if p_from[0] <= p_to[0] else [p_to[0], p_from[0]]
    t2 = [p_from[1], p_to[1]] if p_from[1] <= p_to[1] else [p_to[1], p_from[1]]
    t1 = np.array(t1)  # - self.radius
    t2 = np.array(t2)  # + self.radius
    partial_map = 1 - self.map[t1[0]:t1[1], t2[0]:t2[1]]

    # deal with line
    if partial_map.shape[1] == 0 or partial_map.shape[0] == 0:
      return 0

    if np.array(partial_map).sum() == 0:
      return 1

    # fix boundary issues: -1
    lt = np.array([t2[0],t1[0]])
    p_from_cv = [p_from[1], p_from[0]] - lt
    p_to_cv = [p_to[1], p_to[0]] - lt
    line_msk = cv2.line(np.zeros(partial_map.shape), p_from_cv, p_to_cv, 1, self.radius*2)

    if np.multiply(partial_map, line_msk).sum() > 0:
      # add new node and edge
      return 0
    else:
      # print('[k]', k+1, ':', np.multiply(partial_map, line_msk).sum(), partial_map.shape, ';', p_from_cv, p_to_cv, '\n',  partial_map, '\n', line_msk)
      return 1

  def drawStartEnd(self, p1=1, p2=1, map=None):
    if map is None:
      map = self.mapshow
    if p1:
      cv2.circle(map, self.start_point[0:2], self.radius, (0, 255, 0), -1)
    if p2:
      cv2.circle(map, self.end_point[0:2], self.radius, (255, 0, 0), -1)

  def readPos(self, event, x, y, flags, param):
    if not self.flag_readpos:
      return

    isValid = self.checkPointValid(x, y)
    if event == cv2.EVENT_MOUSEMOVE:
      mapCopy = self.mapshow.copy()
      cv2.circle(mapCopy, (x, y), self.radius, (0, 0, 255) if not isValid else ((0, 255, 0)if self.flag_readpos == 2 else (255, 0, 0)), -1)
      cv2.imshow(self.WIN_NAME, mapCopy)
      # check if in obsticle
    if event == cv2.EVENT_LBUTTONDOWN and isValid:
      self.flag_readpos = max(self.flag_readpos-1, 0)
      if self.flag_readpos == 1:
        self.start_point = [x, y]
        # cv2.circle(self.mapshow, (x, y), self.radius, (0, 255, 0), -1)
        self.drawStartEnd(1, 0)
      else:
        self.end_point = [x, y]
        # cv2.circle(self.mapshow, (x, y), self.radius, (255, 0, 0), -1)
        self.drawStartEnd(0, 1)
      cv2.imshow(self.WIN_NAME, self.mapshow)

  def point2exists(self, point1, trees):
    return np.sum((np.array(trees)[:, 0:2] - np.array(point1[0:2]))**2, axis=1)

  def edge(self, v, k):
    self.edges[int(k)] = int(v)

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

  def rrt(self) -> int:
    self.nodes.append(self.start_point)
    k = 0

    while k <= self.max_edges:
      # check if reach end point
      dist_end = self.point2exists(self.end_point, [self.nodes[-1]])[0]
      if dist_end <= self.threshold_reach**2 and self.checkLineValid(self.nodes[-1], self.end_point, k):
        print('hoohay!')
        self.end_ind = len(self.nodes)-1
        if dist_end > 0:
          # self.edges[self.end_ind+1] = self.end_ind
          self.edge(self.end_ind, self.end_ind+1)
          self.nodes.append(self.end_point)  # rrt*
          self.end_ind += 1
        return 1

      x = random.randrange(0, self.w)
      y = random.randrange(0, self.h)
      if not self.checkPointValid(x, y):
        # TODO return nearest valid point
        continue
      cur_node = [x, y]  # rrt*
      dist_cur = self.point2exists(cur_node, self.nodes)
      ind_near = np.argmin(dist_cur)
      dis_near = dist_cur[ind_near]
      near_node = self.nodes[ind_near]
      # repeat points
      if dis_near <= 0:
        continue
      # check max distance
      if dis_near > self.max_dis**2:
        vec = np.array(cur_node)-np.array(near_node)
        vec = vec/np.linalg.norm(vec)*self.max_dis
        cur_node = near_node + np.array(vec).astype(int)  # rrt*
        if not self.checkPointValid(*cur_node[0:2]):
          continue

      # check if exists obsticles along path
      if self.checkLineValid(near_node, cur_node, k):
        self.connect_edge(ind_near, 0, cur_node)
        k += 1
        # rrt*
      else:
        # TODO return nearest valid point
        pass

    # no valid path found
    return 0

  def connect_edge(self, n1_ind, n2_ind, cur_node=None):
    if cur_node is not None:
      self.nodes.append(cur_node)
      n2_ind = len(self.nodes)-1
      cv2.circle(self.mapshow, cur_node[0:2], 2, (0, 0, 255), -1)

    lst_edge = [n1_ind, n2_ind]
    # self.edges[lst_edge[1]] = lst_edge[0]
    self.edge(*lst_edge)

    # rrt*
    cv2.line(self.mapshow, self.nodes[lst_edge[0]][0:2], self.nodes[lst_edge[1]][0:2], (100, 100, 100))
    # cv2.putText(self.mapshow, str(k), cur_node, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=.3, color=(111, 111, 111))
    cv2.imshow(self.WIN_NAME, self.mapshow)
    return n2_ind

  def pos(self, ind):
    return self.nodes[ind][0:2]

  def findpath(self, cur):
    self.path_inds = []
    self.path = []
    while cur and cur > 0:
      self.path_inds.append(cur)
      self.path.append(self.nodes[cur])
      nxt = self.edges.get(cur)
      cv2.line(self.mapshow, self.pos(cur), self.pos(nxt), (0, 200, 200), round(self.radius/2))
      if nxt > 0:
        cv2.circle(self.mapshow, self.pos(nxt), self.radius, (200, 200, 0), -1)
      else:
        self.path_inds.append(nxt)
        self.path.append(self.nodes[nxt])
      cur = nxt

    cv2.imshow(self.WIN_NAME, self.mapshow)
    self.path_inds.reverse()
    print('----%d----\n' % self.found, self.path_inds)
    self.path.reverse()
    print('----%d----\n' % self.found, self.path)
    cv2.imwrite(self.FILE_PREFIX+'result.png', self.mapshow)
    return self.path_inds

  def smooth_path(self, use_smooth=True):
    mapshowbk = cv2.imread(self.FILE_PREFIX+'mapshowbk.png')
    # map_obsticle_msk = 1-cv2.cvtColor(self.maporigin, cv2.COLOR_BGR2GRAY)//255
    map_obsticle_msk = 1-np.array(self.maporigin)//255
    # map_obsticle_msk = mapshowbk_msk[np.argwhere(mapshowbk_msk == 0)]
    # mapshowbk = self.mapshow

    if use_smooth:
      tck, u1 = splprep(np.array(self.path).T, s=0, k=3)
      u = np.linspace(min(u1), max(u1), num=len(self.path) * self.max_dis // self.radius, endpoint=True)  # len(self.path) * self.max_dis // self.radius
      new_points = splev(u, tck)
      new_points = np.array(new_points).T.round().astype(int)

      print('smoothing...')

      line_start = None
      line_flag = False
      # bad_points = []
      for i in new_points:
        # cv2.line(self.mapshow, i, (100, 100, 0), round(self.radius/2))
        mapcp = mapshowbk.copy()
        cv2.circle(mapcp, i.astype(int), self.radius, (100, 0, 100, 0.5), -1)
        if self.check_obsticals(mapcp, map_obsticle_msk)[0] > 0:
          line_flag = True
          # bad_points.append(i)
        elif line_flag:
          cv2.line(mapcp, line_start.astype(int), i.astype(int), (0, 200, 200), self.radius*2)
          if self.check_obsticals(mapcp, map_obsticle_msk)[0] > 0:
            continue
          line_flag = False
          mapshowbk = mapcp
          line_start = i
        else:
          mapshowbk = mapcp
          line_start = i

      clr = [0, 0, 255]
      err, t = self.check_obsticals(mapshowbk, map_obsticle_msk)
      print('err', err)
      mapshowbk = (mapshowbk*(1-t)+t*clr).astype(np.uint8)
      cv2.imwrite(self.FILE_PREFIX+'smooth_%d.png' % self.found, mapshowbk)
      # exists error or not finished due to error
      if err > 0 or mapshowbk[self.path[-1][1], self.path[-1][0], 2] == 255:
        self.smooth_path(use_smooth=False)
        return
    else:
      print('smoothing fail, fallback to connecting line...')

      colr = (0, 200, 200)
      for i in range(len(self.path)-1):
        if len(self.nodes[0]) > 2:
          dis = self.point2exists(self.nodes[self.path_inds[i+1]], [self.nodes[self.path_inds[i]]])[0]**0.5+self.nodes[self.path_inds[i]][2]
          if abs(self.nodes[self.path_inds[i+1]][2] - dis) > 1:
            colr = (0, 0, 200)
            print(self.nodes[self.path_inds[i+1]], dis)
          else:
            colr = (0, 200, 200)
        cv2.line(mapshowbk, self.path[i][0:2], self.path[i+1][0:2], colr, self.radius*2)

      err, t = self.check_obsticals(mapshowbk, map_obsticle_msk)
      print('err', err)

    # demonstration purpose; not in final path
    # for i in bad_points:
    #   cv2.circle(mapshowbk, i.astype(int), self.radius, (0, 0, 255), -1)
    for i in self.path:
      cv2.circle(mapshowbk, i[0:2], self.radius, (200, 200, 0), -1)

    self.drawStartEnd(map=mapshowbk)
    cv2.imwrite(self.FILE_PREFIX+'result_%d.png' % self.found, mapshowbk)
    cv2.imshow(self.WIN_NAME+'result%d' % self.found, mapshowbk)

  def check_obsticals(self, mapshowbk, map_obsticle_msk):
    t = mapshowbk*map_obsticle_msk
    err = np.sum(t[:, :, 2] > 0)
    return err, t

  def blockui(self):
    print('exit by esc')
    while 1:
      k = cv2.waitKey(1) & 0xFF
      if k == 27:
        break
    cv2.destroyAllWindows()


if __name__ == '__main__':
  # for i in range(3):
  i = 2
  RRT('maps/map%d.png' % (i+1), max_edges=5000)
