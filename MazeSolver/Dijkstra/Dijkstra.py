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


class DijkstraAlgorithm:
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
