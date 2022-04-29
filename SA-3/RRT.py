
import cv2
import numpy as np
import random
import json
from scipy.interpolate import splev, splprep


class RRT:
  WIN_NAME = 'RRT_'
  FILE_PREFIX = 'result/'+WIN_NAME
  DEBUG = False

  def __init__(self, mappath='', maxdistance=50, radius=5, max_edges=10000, collision_dis=1):
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
    self.run()

  def run(self):
    self.loadmap()
    path_exist = False
    if RRT.DEBUG:
      self.mapshow = cv2.imread(self.FILE_PREFIX+'mapshow.png')
      cv2.imshow(RRT.WIN_NAME, self.mapshow)
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
      cv2.imwrite(self.FILE_PREFIX+'mapshow.png', self.mapshow)
      with open(self.FILE_PREFIX+'data.json', 'w') as f:
        json.dump({'path_exist': path_exist, 'start_point': self.start_point, 'end_point': self.end_point}, f)
    if path_exist:
      self.findpath(self.end_ind)
      self.smooth_path()
    self.blockui()

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

  # def point2exists(self, point1, points):
  #   return np.sum((np.array(points) - np.array(point1))**2, axis=1)

  def point2exists(self, point1, trees):
    return np.sum((np.array(trees)[:, 0:2] - np.array(point1[0:2]))**2, axis=1)

  def edge(self, v, k):
    self.edges[int(k)] = int(v)


  def rrt(self) -> int:
    self.nodes.append(self.start_point)
    k = 0

    while k <= self.max_edges:
      # check if reach end point
      dist_end = self.point2exists(self.end_point, [self.nodes[-1]])
      if dist_end <= self.threshold_reach**2:
        print('hoohay!')
        self.end_ind = len(self.nodes)-1
        if dist_end > 0:
          # self.edges[self.end_ind+1] = self.end_ind
          self.edge(self.end_ind, self.end_ind+1)
          self.nodes.append(self.end_point)
          self.end_ind += 1
        return 1

      x = random.randrange(0, self.w)
      y = random.randrange(0, self.h)
      if not self.checkPointValid(x, y):
        # TODO return nearest valid point
        continue
      cur_node = [x, y]
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
        cur_node = near_node + np.array(vec).astype(int)

      # check if exists obsticles along path
      t1 = (near_node[0], cur_node[0]) if near_node[0] <= cur_node[0] else (cur_node[0], near_node[0])
      t2 = (near_node[1], cur_node[1]) if near_node[1] <= cur_node[1] else (cur_node[1], near_node[1])
      partial_map = self.map[t1[0]:t1[1], t2[0]:t2[1]]
      if partial_map.shape[1] == 0 or partial_map.shape[0] == 0:
        # TODO deal with line
        continue

      # fix boundary issues: -1
      line_msk = cv2.line(np.zeros(partial_map.shape), (0, 0), (partial_map.shape[1]-1, partial_map.shape[0]-1), self.radius*2, 1)

      # print('[k]', k+1, ':', partial_map, line_msk, partial_map.shape, t1, t2)

      if line_msk.sum() == np.multiply(partial_map, line_msk).sum():
        # self.linkrrt(cur_node, ind_near, dist_cur)
        # add new node and edge
        self.nodes.append(cur_node)
        lst_edge = [ind_near, len(self.nodes)-1]
        # self.edges[lst_edge[1]] = lst_edge[0]
        self.edge(*lst_edge)
        k += 1

        cv2.circle(self.mapshow, cur_node, 2, (0, 0, 255), -1)
        cv2.line(self.mapshow, self.nodes[lst_edge[0]], self.nodes[lst_edge[1]], (100, 100, 100))
        # cv2.putText(self.mapshow, str(k), cur_node, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=.3, color=(111, 111, 111))

        # Using cv2.putText() method
        cv2.imshow(RRT.WIN_NAME, self.mapshow)
      else:
        # TODO return nearest valid point
        pass

    # no valid path found
    return 0

  def pos(self, ind):
    return self.nodes[ind][0:2]

  def findpath(self, cur):
    # paths = []
    while cur and cur > 0:
      # paths.append(cur)
      self.path.append(self.nodes[cur])
      nxt = self.edges.get(cur)
      cv2.line(self.mapshow, self.pos(cur), self.pos(nxt), (0, 200, 200), round(self.radius/2))
      if nxt > 0:
        cv2.circle(self.mapshow, self.pos(nxt), self.radius, (200, 200, 0), -1)
      else:
        # paths.append(nxt)
        self.path.append(self.nodes[nxt])
      cur = nxt

    cv2.imshow(RRT.WIN_NAME, self.mapshow)
    # paths.reverse()
    # print(paths)
    self.path.reverse()
    print(self.path)
    cv2.imwrite(self.FILE_PREFIX+'result.png', self.mapshow)

  def line_path(self):
    mapshowbk = cv2.imread(RRT.FILE_PREFIX+'mapshowbk.png')
    map_obsticle_msk = 1-np.array(self.maporigin)//255

    print('smoothing fail, fallback to connecting line...')

    for i in range(len(self.path)-1):
      cv2.line(mapshowbk, self.path[i], self.path[i+1], (0, 200, 200), self.radius*2)

    clr = [0, 0, 255]
    err, t = self.check_obsticals(mapshowbk, map_obsticle_msk)
    print('err', err)
    # mapshowbk = (mapshowbk*(1-t)+t*clr).astype(np.uint8)

    # demonstration purpose; not in final path
    for i in self.path:
      cv2.circle(mapshowbk, i, self.radius, (200, 200, 0), -1)
    cv2.imshow(RRT.WIN_NAME+'smooth', mapshowbk)
    cv2.imwrite(RRT.FILE_PREFIX+'line.png', mapshowbk)

  def smooth_path(self):
    mapshowbk = cv2.imread(RRT.FILE_PREFIX+'mapshowbk.png')
    # map_obsticle_msk = 1-cv2.cvtColor(self.maporigin, cv2.COLOR_BGR2GRAY)//255
    map_obsticle_msk = 1-np.array(self.maporigin)//255
    # map_obsticle_msk = mapshowbk_msk[np.argwhere(mapshowbk_msk == 0)]
    # mapshowbk = self.mapshow
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
    cv2.imwrite(RRT.FILE_PREFIX+'smooth.png', mapshowbk)
    # exists error or not finished due to error
    if err > 0 or mapshowbk[self.path[-1][1], self.path[-1][0], 2] == 255:
      self.line_path()
      return

    # demonstration purpose; not in final path
    # for i in bad_points:
    #   cv2.circle(mapshowbk, i.astype(int), self.radius, (0, 0, 255), -1)
    for i in self.path:
      cv2.circle(mapshowbk, i, self.radius, (200, 200, 0), -1)

    cv2.imshow(RRT.WIN_NAME+'smooth', mapshowbk)

  def check_obsticals(self, mapshowbk, map_obsticle_msk):
    t = mapshowbk*map_obsticle_msk
    err = np.sum(t[:, :, 2] > 0)
    return err, t

  def blockui(self):
    while 1:
      k = cv2.waitKey(1) & 0xFF
      if k == 27:
          break
    cv2.destroyAllWindows()


if __name__ == '__main__':
  # for i in range(3):
  i = 2
  RRT('maps/map%d.png' % (i+1), max_edges=10000)
