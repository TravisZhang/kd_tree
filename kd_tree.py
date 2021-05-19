import math
import copy


class Queue():
    def __init__(self):
        self.q = list()

    def enq(self, v):
        self.q.append(v)

    def deq(self):
        if len(self.q) > 0:
            return self.q.pop(0)
        else:
            return None

    def empty(self):
        return len(self.q) == 0

    def __len__(self):
        return len(self.q)


class Stack():
    def __init__(self):
        self.s = list()

    def push(self, v):
        self.s.append(v)

    def pop(self):
        if len(self.s) > 0:
            return self.s.pop()
        else:
            return None

    def top(self):
        if len(self.s) > 0:
            return self.s[-1]
        else:
            return None

    def empty(self):
        return len(self.s) == 0

    def __len__(self):
        return len(self.s)


class Node:
    def __init__(self):
        self.range = None  # space range
        self.data = None  # k dimensional vector
        self.split = None  # the index of axis vertical to the split hyper surface
        self.left = None  # left child kd tree
        self.right = None  # right child kd tree
        self.parent = None  # parent node

    def SetParent(self, node):
        self.parent = node

    def SetLeftChild(self, node):
        self.left = node

    def SetRightChild(self, node):
        self.right = node

    def SetSplit(self, split):
        self.split = split

    def SetData(self, data):
        self.data = data

    def Data(self):
        return self.data

    def LeftChild(self):
        return self.left

    def RightChild(self):
        return self.right

    def Parent(self):
        return self.parent

    def HasLeftChild(self):
        return self.left != None

    def HasRightChild(self):
        return self.right != None

    def HasParent(self):
        return self.parent != None

    def Split(self):
        return self.split

    def HasSplit(self):
        return self.split != None

    def HasData(self):
        return self.data != None

    def IsLeftChild(self, node):
        if self.split == None or not node.HasData():
            return False
        return self.data[self.split] > node.Data()[self.split]

    def IsRightChild(self, node):
        if self.split == None or not node.HasData():
            return False
        return self.data[self.split] <= node.Data()[self.split]

    def IsLeaf(self):
        return not (self.HasLeftChild() or self.HasRightChild())


class KDTree:
    def __init__(self, data_set=None):
        self.root = Node()
        self.data_set = data_set
        self.dimension = None
        if data_set is not None and len(data_set) > 0:
            self.dimension = len(self.data_set[0])
            self.root = self.CreateNode(self.data_set)

    # maybe we have better method for dividing data
    def CalculateVariation(self, data_set, cur_dim):
        if len(data_set) == 0 or data_set is None:
            return 0
        mean = 0
        for data in data_set:
            mean += data[cur_dim]
        mean /= len(data_set)
        var = 0
        for data in data_set:
            var += math.pow(data[cur_dim]-mean, 2)
        var /= len(data_set)
        return var

    def InsertSort(self, data_set, cur_dim):
        if len(data_set) < 2:
            return
        for i in range(1, len(data_set)):
            key = copy.deepcopy(data_set[i])
            j = i-1
            while j >= 0 and data_set[j][cur_dim] > key[cur_dim]:
                data_set[j+1] = copy.deepcopy(data_set[j])
                j -= 1
            if j < i-1:
                data_set[j+1] = key

    def CreateNode(self, data_set, depth=0):
        node = Node()
        if len(data_set) == 0:
            print('data len=0, returning')
            return node
        if len(data_set) == 1:
            node.SetData(data_set[0])
            print('data: ', node.Data(), ' depth: ', depth)
            return node
        # determine split with largest variation
        split = 0
        max_var = 0
        for i in range(0, self.dimension):
            var = self.CalculateVariation(data_set, i)
            if var >= max_var:
                max_var = var
                split = i
        print('data len: ', len(data_set), 'split: ', split, ' depth: ', depth)
        node.SetSplit(split)
        # sort data set by split
        # and set mid value as node data
        self.InsertSort(data_set, split)
        mid_idx = int(len(data_set)/2)
        mid_data = data_set[mid_idx]
        node.SetData(mid_data)
        print('sorted set: ', data_set, 'mid idx: ',
              mid_idx, 'mid data', mid_data)
        # get left & right node
        depth += 1
        if mid_idx > 0:
            left_data_set = data_set[0:mid_idx]
            left_node = self.CreateNode(left_data_set, copy.deepcopy(depth))
            left_node.SetParent(node)
            node.SetLeftChild(left_node)
        if mid_idx+1 < len(data_set):
            right_data_set = data_set[mid_idx+1:len(data_set)]
            right_node = self.CreateNode(right_data_set, copy.deepcopy(depth))
            right_node.SetParent(node)
            node.SetRightChild(right_node)

        return node

    def PrintTree(self, print_type=1):
        print('printing tree ...')
        # use bfs/dfs to print tree
        if print_type == 1:
            q = Queue()
            q.enq(self.root)
            # print('level: ',step,' node: ',self.root.Data())
            while not q.empty():
                current_node = q.deq()
                print('current node: ', current_node.Data(),
                      ' split: ', current_node.Split())
                if current_node.HasLeftChild():
                    q.enq(current_node.LeftChild())
                if current_node.HasRightChild():
                    q.enq(current_node.RightChild())
        else:

            def dfs(node, s):
                s.push(node)
                step = 0
                if node.HasLeftChild():
                    dfs(node.LeftChild(), s)
                step += 1
                if node.HasRightChild():
                    dfs(node.RightChild(), s)
                step += 1
                # we need to print leaf node
                # and node that all its children has been printed
                if not(node.HasLeftChild() or node.HasRightChild()) or step == 2:
                    print('current node: ', node.Data())
                    s.pop()

            s = Stack()
            dfs(self.root, s)

        print('print tree done')

    def FindNearest(self, node, search_radius):
        def CalDist(n0, n1):  # L2 norm(maybe other norm possible?)
            sqr_sum = 0
            for i in range(0, len(n0.Data())):
                deviation = n0.Data()[i]-n1.Data()[i]
                sqr_sum += math.pow(deviation, 2)
            return math.sqrt(sqr_sum)

        def JudgeCutWithHyperSurface(node, target_node, min_dist):
            # judge a circle/ball/hypersphere ... with node as center, radius as radius
            # has intersection with hyper surface that is vertical to given axis
            # first project node onto surface
            radius = min(CalDist(node, target_node), min_dist)
            surface_axis = target_node.Split()
            surface_dist_to_origin = target_node.Data()[surface_axis]
            projection_node = Node()
            projection_node_data = copy.deepcopy(node.Data())
            projection_node_data[surface_axis] = surface_dist_to_origin
            projection_node.SetData(projection_node_data)
            # check if dist btwn two nodes is bigger than radius
            projection_dist = CalDist(node, projection_node)
            # method 2
            projection_dist = abs(
                node.Data()[surface_axis]-target_node.Data()[surface_axis])
            print('axis: ', surface_axis, ' dist to origin: ', surface_dist_to_origin,
                  ' surface dist:', projection_dist, ' radius: ', radius)
            if projection_dist > radius:
                return False
            return True

        # binary search & record search path
        current_node = self.root
        node_list = Stack()
        node_list.push(current_node)
        node_data_list = [current_node.Data()]
        print('current node has left: ', current_node.HasLeftChild())
        while (current_node.HasLeftChild() or current_node.HasRightChild()):
            print('current node: ', current_node.Data(),
                  ' split: ', current_node.Split())
            if not current_node.HasSplit():
                break
            current_split = current_node.Split()
            if current_node.Data()[current_split] > node.Data()[current_split]:
                current_node = current_node.LeftChild()
            else:
                current_node = current_node.RightChild()
            if CalDist(node, current_node) < search_radius:
                node_list.push(current_node)
                node_data_list.append(current_node.Data())
        print('node list: ', node_data_list)

        # reverse search to update possible closer node
        nearest_node = node_list.pop()
        min_dist = CalDist(node, nearest_node)

        print('begin reverse search, nearest:', nearest_node.Data())
        while not node_list.empty():
            print('node list len: ', len(node_list))
            target_node = node_list.pop()
            print('target node: ', target_node.Data(),
                  ' split: ', target_node.Split())
            # update nearest node
            temp_dist = CalDist(node, target_node)
            if min_dist > temp_dist:
                min_dist = temp_dist
                nearest_node = target_node
                print('update nearest: ', nearest_node.Data())
            # check the other branch
            if not target_node.IsLeaf():
                if not JudgeCutWithHyperSurface(node, target_node, min_dist):
                    # if node doesnt intersect with target node's hyper surface
                    # reverse search is done(if it does, there's a chance the other side
                    # has closer dist node)
                    print('no cut break')
                    break
                print('cut with surface')
                if target_node.IsLeftChild(node) and target_node.HasRightChild():
                    next_node = target_node.RightChild()
                    # push current node in node list
                    node_list.push(next_node)
                elif target_node.HasLeftChild():
                    next_node = target_node.LeftChild()
                    # push current node in node list
                    node_list.push(next_node)
                

        return nearest_node.Data()

    # Note this is NOT recommanded because
    # Adding points in this manner can cause the tree to become unbalanced, leading to decreased tree performance. The rate of tree performance degradation is dependent upon the spatial distribution of tree points being added, and the number of points added in relation to the tree size
    def AddNode(self, node):
         # binary search & record search path
        current_node = self.root
        node_list = Stack()
        node_list.push(current_node)
        node_data_list = [current_node.Data()]
        print('current node has left: ', current_node.HasLeftChild())
        while (current_node.HasLeftChild() or current_node.HasRightChild()):
            print('current node: ', current_node.Data(),
                  ' split: ', current_node.Split())
            if not current_node.HasSplit():
                break
            current_split = current_node.Split()
            if current_node.Data()[current_split] > node.Data()[current_split]:
                current_node = current_node.LeftChild()
            else:
                current_node = current_node.RightChild()
            node_list.push(current_node)
            node_data_list.append(current_node.Data())
        print('node list: ', node_data_list)

        if current_node.HasSplit():
            current_split = current_node.Split()
            if current_node.Data()[current_split] > node.Data()[current_split]:
                current_node.SetLeftChild(node)
            else:
                current_node.SetRightChild(node)
            node.SetParent(current_node)


data_set = [[2, 3], [5, 4], [9, 6], [4, 7], [8, 1], [7, 2]]
# data_set = [[19, 2], [7, 0], [13, 5], [3, 15], [3, 4], [3, 2], [8, 9], [9, 3], [17, 15], [11, 11]]
kd_tree = KDTree(data_set)
kd_tree.PrintTree()
target_node = Node()
# target_node.SetData([2.1,3.1])
target_node.SetData([2, 4.5])
# target_node.SetData([12, 7])
print('nearest node: ', kd_tree.FindNearest(target_node, 100))
