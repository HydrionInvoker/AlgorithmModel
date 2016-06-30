#! usr/bin/env python
#coding=utf-8
#网上搜索到的A*路径规划算法，可直接运行
import math
import time
#地图


tm = [
'############################################################',
'#..........................................................#',
'#.............................#............................#',
'#.............................#............................#',
'#.....................E.......#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#.............................#............................#',
'#######.#######################################............#',
'#....#........#............................................#',
'#....#........#............................................#',
'#....##########............................................#',
'#..........................................................#',
'#..........................................................#',
'#..........................................................#',
'#..........................................................#',
'#..........................................................#',
'#...............................##############.............#',
'#...............................#............#.............#',
'#...............................#............#.............#',
'#...............................#.....S......#.............#',
'#...............................#............#.............#',
'#...............................###########..#.............#',
'#..........................................................#',
'#..........................................................#',
'############################################################']

#因为python里string不能直接改变某一元素，所以用test_map来存储搜索时的地图
test_map = []

#########################################################
class Node_Elem:
    """
    开放列表和关闭列表的元素类型，parent用来在成功的时候回溯路径
    """
    def __init__(self, parent, x, y, dist):
        self.parent = parent
        self.x = x
        self.y = y
        self.dist = dist

class A_Star:
    """
    A星算法实现类
    """
    #注意w,h两个参数，如果你修改了地图，需要传入一个正确值或者修改这里的默认参数
    def __init__(self, s_x, s_y, e_x, e_y, w=60, h=30):
        self.s_x = s_x
        self.s_y = s_y
        self.e_x = e_x
        self.e_y = e_y

        self.width = w
        self.height = h

        self.open = []
        self.close = []
        self.path = []

    #查找路径的入口函数
    def find_path(self):
        #构建开始节点
        p = Node_Elem(None, self.s_x, self.s_y, 0.0)
        while True:
            #扩展F值最小的节点
            self.extend_round(p)
            #如果开放列表为空，则不存在路径，返回
            if not self.open:
                return
            #获取F值最小的节点
            idx, p = self.get_best()
            #找到路径，生成路径，返回
            if self.is_target(p):
                self.make_path(p)
                return
            #把此节点压入关闭列表，并从开放列表里删除
            self.close.append(p)
            del self.open[idx]

    def make_path(self,p):
        #从结束点回溯到开始点，开始点的parent == None
        while p:
            self.path.append((p.x, p.y))
            p = p.parent

    def is_target(self, i):
        return i.x == self.e_x and i.y == self.e_y

    def get_best(self):
        best = None
        bv = 1000000 #如果你修改的地图很大，可能需要修改这个值
        bi = -1
        for idx, i in enumerate(self.open):
            value = self.get_dist(i)#获取F值
            if value < bv:#比以前的更好，即F值更小
                best = i
                bv = value
                bi = idx
        return bi, best

    def get_dist(self, i):
        # F = G + H
        # G 为已经走过的路径长度， H为估计还要走多远
        # 这个公式就是A*算法的精华了。
        return i.dist + math.sqrt(
            (self.e_x-i.x)*(self.e_x-i.x)
            + (self.e_y-i.y)*(self.e_y-i.y))*1.2

    def extend_round(self, p):
        #可以从8个方向走
        xs = (-1, 0, 1, -1, 1, -1, 0, 1)
        ys = (-1,-1,-1,  0, 0,  1, 1, 1)
        #只能走上下左右四个方向
#        xs = (0, -1, 1, 0)
#        ys = (-1, 0, 0, 1)
        for x, y in zip(xs, ys):
            new_x, new_y = x + p.x, y + p.y
            #无效或者不可行走区域，则勿略
            if not self.is_valid_coord(new_x, new_y):
                continue
            #构造新的节点
            node = Node_Elem(p, new_x, new_y, p.dist+self.get_cost(
                        p.x, p.y, new_x, new_y))
            #新节点在关闭列表，则忽略
            if self.node_in_close(node):
                continue
            i = self.node_in_open(node)
            if i != -1:
                #新节点在开放列表
                if self.open[i].dist > node.dist:
                    #现在的路径到比以前到这个节点的路径更好~
                    #则使用现在的路径
                    self.open[i].parent = p
                    self.open[i].dist = node.dist
                continue
            self.open.append(node)

    def get_cost(self, x1, y1, x2, y2):
        """
        上下左右直走，代价为1.0，斜走，代价为1.4
        """
        if x1 == x2 or y1 == y2:
            return 1.0
        return 1.4

    def node_in_close(self, node):
        for i in self.close:
            if node.x == i.x and node.y == i.y:
                return True
        return False

    def node_in_open(self, node):
        for i, n in enumerate(self.open):
            if node.x == n.x and node.y == n.y:
                return i
        return -1

    def is_valid_coord(self, x, y):
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return test_map[y][x] != '#'

    def get_searched(self):
        l = []
        for i in self.open:
            l.append((i.x, i.y))
        for i in self.close:
            l.append((i.x, i.y))
        return l

    #########################################################
class Handle_Data:
    """handle external content of the programme"""
    @staticmethod
    def print_test_map():
        """
        打印搜索后的地图
        """
        for line in test_map:
            print ''.join(line)

    @staticmethod
    def get_start_XY():
        return Handle_Data.get_symbol_XY('S')

    @staticmethod
    def get_end_XY():
        return Handle_Data.get_symbol_XY('E')

    @staticmethod
    def get_symbol_XY(s):
        for y, line in enumerate(test_map):
            try:
                x = line.index(s)
            except:#捕获所有异常，进入该代码块处理
                continue
            else:
                break
        return x, y

    #########################################################
    @staticmethod
    def mark_path(l):
        Handle_Data.mark_symbol(l, '*')

    @staticmethod
    def mark_searched(l):
        Handle_Data.mark_symbol(l, ' ')

    @staticmethod
    def mark_symbol(l, s):
        for x, y in l:
            test_map[y][x] = s

    @staticmethod
    def mark_start_end(s_x, s_y, e_x, e_y):
        test_map[s_y][s_x] = 'S'
        test_map[e_y][e_x] = 'E'

    @staticmethod
    def tm_to_test_map():
        for line in tm:
            test_map.append(list(line))

    @staticmethod
    def find_path():
        s_x, s_y = Handle_Data.get_start_XY()
        e_x, e_y = Handle_Data.get_end_XY()
        a_star = A_Star(s_x, s_y, e_x, e_y)
        a_star.find_path()
        searched = a_star.get_searched()
        path = a_star.path
        #标记已搜索区域
        Handle_Data.mark_searched(searched)
        #标记路径
        Handle_Data.mark_path(path)
        print "path length is %d"%(len(path))
        print "searched squares count is %d"%(len(searched))
        #标记开始、结束点
        Handle_Data.mark_start_end(s_x, s_y, e_x, e_y)
        return path

if __name__ == "__main__":
    #把字符串转成列表
    Handle_Data.tm_to_test_map()
    path = Handle_Data.find_path()
    Handle_Data.print_test_map()
    pass