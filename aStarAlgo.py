"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

"""
追加機能
- NGリスト
- メッシュサイズ
- 解なし判定
- メッシュIDを出力
"""
import math


class AStarPlanner:

    def __init__(self, ox, oy, reso, rr, ms):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        ms: mesh size[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model(ms)

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart


        while 1:
            # スタートからゴールまで経路が結べない
            if len(open_set) == 0:
                #print("NotFound")
                return [], [], 0

            c_id = min(
                open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            if current.x == ngoal.x and current.y == ngoal.y:
                #print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            #print(self.motion)
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)

                n_id = self.calc_grid_index(node)


                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry, ngoal.cost

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            # rx, ryに入るのはゴールまでの経路
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 2.0  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position
        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.sqrt((iox - x) ** 2 + (ioy - y) ** 2)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model(ms):
        # dx, dy, cost
        diagonalMoveCost = math.sqrt(2) * ms
        motion = [[1, 0, ms],
                  [0, 1, ms],
                  [-1, 0, ms],
                  [0, -1, ms],
                  [-1, -1, diagonalMoveCost],
                  [-1, 1, diagonalMoveCost],
                  [1, -1, diagonalMoveCost],
                  [1, 1, diagonalMoveCost]]

        return motion

class routeSearch:
    def __init__(self, meshSize):
        # 5km四方で考える
        limit = 5000  # 5000m
        self.meshNum = int(limit / meshSize) # 1辺の個数
        self.meshSize = meshSize # 1辺の大きさ

    def main(self, sx, sy, gx, gy, ngList):
        #print(" start!!")

        # スタートまたはゴールがNGと重なってる
        sID = sx + (sy*self.meshNum)
        gID = gx + (gy*self.meshNum)
        #print("sID: ", sID)
        #print("gID: ", gID)
        #print("ng: ", ngList)
        if (sID in ngList) or (gID in ngList):
            return [], 0

        # スタートとゴールが同じメッシュ
        if sID == gID:
            return [[sID]], -1
            

        # start and goal position
        #sx = sx  # [m]
        #sy = sy  # [m]
        #gx = gx  # [m]
        #gy = gy  # [m]
        grid_size = 1.0    # [m]
        robot_radius = 0.5 # [m]

        # set obstable positions
        ox, oy = [], []
        for i in range(-1, self.meshNum):
            ox.append(i)
            oy.append(-1.0)
        for i in range(-1, self.meshNum):
            ox.append(self.meshNum)
            oy.append(i)
        for i in range(-1, self.meshNum+1):
            ox.append(i)
            oy.append(self.meshNum)
        for i in range(-1, self.meshNum+1):
            ox.append(-1.0)
            oy.append(i)

        # NGのメッシュ
        for mID in ngList:
            ox.append(mID % self.meshNum)
            oy.append(mID // self.meshNum)

        a_star = AStarPlanner(ox, oy, grid_size, robot_radius, self.meshSize)
        rx, ry, cost = a_star.planning(sx, sy, gx, gy) # スタートからゴールまでの経路

        # メッシュID化
        meshID = []
        for x, y in zip(rx, ry):
            meshID.insert(0, [int(x + (y * self.meshNum))])

        return meshID, cost

#ngList = []
#test = routeSearch(100)
#m, c = test.main(0, 0, 5, 0, ngList)
#print(m)
#print(c)