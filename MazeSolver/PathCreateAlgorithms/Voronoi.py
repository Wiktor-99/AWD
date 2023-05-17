from scipy.spatial import cKDTree, Voronoi
import numpy as np
import math
from Dijkstra.Dijkstra import DijkstraAlgorithm

class VornoiPathFinder:
    def __init__(self, start, end, points):
        self.startX = start[0]
        self.startY = start[1]
        self.endX = end[0]
        self.endY = end[1]
        self.points = points
        self.robotRadius = round(10 * 2**(0.5))
        self.obstacleRadius = 30

    def isCollision(self, startX, startY, endX, endY,obstacle_kd_tree, edge_length=50):
        x =  startX
        y = startY
        yaw = math.atan2(endY - y, endX - x)
        d = math.hypot(endX - x, endY - y)
        collision_dist = self.robotRadius + self.obstacleRadius

        if d >= edge_length:
            return True

        n_step = round(d / self.robotRadius)

        for i in range(n_step):
            dist, _ = obstacle_kd_tree.query([x, y])
            if dist <= collision_dist:
                return True
            x += self.robotRadius  * math.cos(yaw)
            y += self.robotRadius  * math.sin(yaw)

        dist, _ = obstacle_kd_tree.query([self.endX, self.endY])
        if dist <= collision_dist:
            return True

        return False


    def voronoiSampling(self):
        oxy = np.array(self.points)

        vor = Voronoi(oxy, incremental=True)
        sample_x = [ix for [ix, _] in vor.vertices]
        sample_y = [iy for [_, iy] in vor.vertices]

        sample_x.append(self.startX)
        sample_y.append(self.startY)
        sample_x.append(self.endX)
        sample_y.append(self.endY)

        return sample_x, sample_y, vor


    def generateRoadMapInfo(self, node_x, node_y, obstacle_tree, edge_length):
        road_map = []
        n_sample = len(node_x)
        node_tree = cKDTree(np.vstack((node_x, node_y)).T)

        for (i, ix, iy) in zip(range(n_sample), node_x, node_y):
            _, indexes = node_tree.query([ix, iy], k=n_sample)
            edge_id = []
            for ii in range(1, len(indexes)):
                nx = node_x[indexes[ii]]
                ny = node_y[indexes[ii]]

                if not self.isCollision(ix, iy, nx, ny, obstacle_tree, edge_length):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= 5000:
                    break

            road_map.append(edge_id)

        return road_map

    def countPathLength(self, path):
        total_length = 0
        for i in range(len(path)):
            if i + 1 >= len(path):
                break
            total_length += ((path[i+1][0] - path[i][0]) **
                            2 + (path[i+1][1] - path[i][1])**2)**(1/2)

        return total_length

    def findVoronoiPath(self):
        obstacle_tree = cKDTree(np.array(self.points))
        sample_x, sample_y, _ = self.voronoiSampling()
        edge_length = 120
        map_info = self.generateRoadMapInfo(sample_x, sample_y, obstacle_tree, edge_length)

        rx, ry = DijkstraAlgorithm().search(
            self.startX, self.startY, self.endX, self.endY, sample_x, sample_y, map_info)

        return list(zip(rx, ry))