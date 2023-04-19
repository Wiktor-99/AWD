from scipy.spatial import cKDTree, Voronoi
import numpy as np
import math


def is_same_node_with_xy(node_x, node_y, node_b):
    dist = np.hypot(node_x - node_b.x,
                    node_y - node_b.y)
    return dist <= 0.1

def is_same_node(node_a, node_b):
    dist = np.hypot(node_a.x - node_b.x,
                    node_a.y - node_b.y)
    return dist <= 0.1


class Dijkstra:
    class Node:
        def __init__(self, x, y, cost=None, parent=None, edge_ids=None):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent = parent
            self.edge_ids = edge_ids


    def search(self, sx, sy, gx, gy, node_x, node_y, edge_ids_list):
        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)
        current_node = None

        open_set, close_set = dict(), dict()
        open_set[self.find_id(node_x, node_y, start_node)] = start_node

        while True:
            if self.has_node_in_set(close_set, goal_node):
                goal_node.parent = current_node.parent
                goal_node.cost = current_node.cost
                break
            elif not open_set:
                break

            current_id = min(open_set, key=lambda o: open_set[o].cost)
            current_node = open_set[current_id]


            del open_set[current_id]
            close_set[current_id] = current_node

            for i in range(len(edge_ids_list[current_id])):
                n_id = edge_ids_list[current_id][i]
                dx = node_x[n_id] - current_node.x
                dy = node_y[n_id] - current_node.y
                d = math.hypot(dx, dy)
                node = self.Node(node_x[n_id], node_y[n_id], current_node.cost + d, current_id)

                if n_id in close_set:
                    continue

                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
                else:
                    open_set[n_id] = node


        rx, ry = self.generate_final_path(close_set, goal_node)

        return rx, ry

    def generate_final_path(self, close_set, goal_node):
        rx, ry = [goal_node.x], [goal_node.y]
        parent = goal_node.parent
        while parent != -1:
            n = close_set[parent]
            rx.append(n.x)
            ry.append(n.y)
            parent = n.parent
        rx, ry = rx[::-1], ry[::-1]
        return rx, ry

    def has_node_in_set(self, target_set, node):
        for key in target_set:
            if is_same_node(target_set[key], node):
                return True
        return False

    def find_id(self, node_x_list, node_y_list, target_node):
        for i, _ in enumerate(node_x_list):
            if is_same_node_with_xy(node_x_list[i], node_y_list[i],
                                         target_node):
                return i
        return None



class VornoiPathFinder:
    def __init__(self, start, end, points):
        self.startX = start[0]
        self.startY = start[1]
        self.endX = end[0]
        self.endY = end[1]
        self.points = points
        self.robotRadius = 10
        self.obstacleRadius = 10

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
        path = []
        edge_len = 120
        map_info = self.generateRoadMapInfo(sample_x, sample_y, obstacle_tree, edge_len)
        rx, ry = Dijkstra().search(self.startX, self.startY, self.endX, self.endY, sample_x, sample_y, map_info)
        path = list(zip(rx, ry))

        return path