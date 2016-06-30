#! /usr/bin/env python
#coding:utf-8

#A*路径规划算法（by Hydrion）
import copy
import math

"""由激光雷达数据实现动态路径规划功能"""

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
'#...............................#......S.....#.............#',
'#...............................#............#.............#',
'#...............................#............#.............#',
'#...............................###########..#.............#',
'#..........................................................#',
'#..........................................................#',
'############################################################']

class PointProperty:
    """某一点的所有数据，x,y坐标，距离起点的步数g和预估到达终点距离h"""
    def __init__(self,x = 0,y = 0,bx = -1,by = -1,g = 0,h = 0,flag = False):
        #初始化目前点的x.y坐标
        self.x = x
        self.y = y
        #初始化父节点的x,y坐标
        self.bx = bx
        self.by = by
        #初始化点的已行距离和距离终点的预估距离
        self.g = g
        self.h = h
        self.flag = flag

class ListProperty:
    """A*算法需要使用的表结构，开表O，闭表C和路径列表Route"""
    o = []#通过元组的形式来保存开表和闭表的数据，元组的内容分别为（点x坐标，点y坐标，预计总距离，父点x坐标，父点y坐标）
    c = []
    route = []

class Pre_Handle:

    map_matrix = None
    def __init__(self):
        # self.save_map_data()
        self.output_map(tm)
        pass

    @staticmethod
    def output_map(tm):
        """分行输出地图"""
        print "new map is "
        for i in range(0,len(tm)):
            for j in range(0,len(tm[i])):
                print tm[i][j],
            print

    @staticmethod
    def get_start(map_matrix):
        """获得起点坐标"""
        for i in range(0,len(map_matrix)):
            try:
                sta_x = map_matrix[i].index("S")
                sta_y = i
                break
            except:
                continue
        return (sta_x,sta_y)

    @staticmethod
    def get_end(map_matrix):
        """获得终点坐标"""
        for i in range(0,len(map_matrix)):
            try:
                end_x = map_matrix[i].index("E")
                end_y = i
                break
            except:
                continue
        return (end_x,end_y)


class RouteAlgorithm:
    """路径规划算法主体"""
    def __init__(self):
        """设定起点，终点和A-star算法的重要参数"""
        self.map_matrix = []
        for line in tm: #将地图数据以二维数组的形式存储以便之后处理
            self.map_matrix.append(list(line))

        (self.sta_x,self.sta_y) = Pre_Handle.get_start(self.map_matrix)
        (self.end_x,self.end_y) = Pre_Handle.get_end(self.map_matrix)

        #初始化当前位置点
        self.pp = PointProperty()
        self.pp.x = self.sta_x
        self.pp.y = self.sta_y
        self.pp.g = 0
        self.pp.h = self.get_estimate_distance(self.pp)
        ListProperty.o.append(self.pp)
        print "初始化成功，起点坐标和终点坐标分别为(%d,%d),(%d,%d)"%(self.sta_x,self.sta_y,self.end_x,self.end_y)

    def main(self):
        """算法的调用主体"""
        while not (self.pp.x == self.end_x and self.pp.y == self.end_y):

            self.move_another_grid()# 向周围的点进行移动，评估其到终点的距离

            shortest_sequence = self.find_shortest_path()# 到开表中寻找已搜索的预估距离最短点进行下一步移动
            self.pp = ListProperty.o[shortest_sequence]

        self.write_the_route()# 将搜索完成的路径放入route列表中

        self.output_the_route()#输出路径规划后的地图

    def find_shortest_path(self):
        """寻找返回预估距离最短点位于开表中的位置"""
        shortest_f = 1000000
        sequence = -1 #保存的距离最短点的标号
        for i in range(0,len(ListProperty.o)):
            if ListProperty.o[i].h+ListProperty.o[i].g < shortest_f:
                shortest_f = ListProperty.o[i].h+ListProperty.o[i].g
                sequence = i
        return sequence

    def get_estimate_distance(self,pp):
        """计算当前点和终点的预估距离G"""
        dis_x = math.fabs(self.end_x - pp.x)
        dis_y = math.fabs(self.end_y - pp.y)
        distance = dis_x + dis_y
        return distance

    def get_est_whole_distrance(self,pp):
        """计算经过该点起点到终点的预估距离"""
        whole_distance = pp.g+self.get_estimate_distance(pp)
        return whole_distance

    def move_another_grid(self):
        """移动至下一个点"""
        UP = [0,1]
        DOWN = [0,-1]
        RIGHT = [1,0]
        LEFT = [-1,0]

        UP_RIGHT = [1,1]
        UP_LEFT = [-1,1]
        DOWN_LEFT = [-1,-1]
        DOWN_RIGHT = [1,-1]

        if self.is_passable(UP) == True and self.in_close_list(UP) == False:
            self.deal_step_point(UP)

        if self.is_passable(DOWN) == True and self.in_close_list(DOWN) == False:
            self.deal_step_point(DOWN)

        if self.is_passable(RIGHT) == True and self.in_close_list(RIGHT) == False:
            self.deal_step_point(RIGHT)

        if self.is_passable(LEFT) == True and self.in_close_list(LEFT) == False:
            self.deal_step_point(LEFT)

        if self.is_passable(UP_RIGHT) == True and self.in_close_list(UP_RIGHT) == False:
            self.deal_step_point(UP_RIGHT)

        if self.is_passable(UP_LEFT) == True and self.in_close_list(UP_LEFT) == False:
            self.deal_step_point(UP_LEFT)

        if self.is_passable(DOWN_LEFT) == True and self.in_close_list(DOWN_LEFT) == False:
            self.deal_step_point(DOWN_LEFT)

        if self.is_passable(DOWN_RIGHT) == True and self.in_close_list(DOWN_RIGHT) == False:
            self.deal_step_point(DOWN_RIGHT)

        #完成搜索后将原先的父点放入close表中
        try:
            cur_index = ListProperty.o.index(self.pp)
            cur_point = ListProperty.o.pop(cur_index)
            ListProperty.c.append(cur_point)
        except:
            pass
    def deal_step_point(self,*location):
        """将当前行进过程中的点进行处理"""

        pp_next = PointProperty(self.pp.x + location[0][0],self.pp.y + location[0][1], self.pp.x , self.pp.y , self.pp.g+1 ,self.pp.h )# 根据下一个点的信息情况创建下一个点的对象

        pp_next.h = self.get_estimate_distance(pp_next)#计算该点的h值然后修改

        #对该点是否在开表中或闭表中进行判断，并执行相应的处理方式
        if self.in_open_list(pp_next) == True:
            self.judge_est_distance(pp_next)
        elif self.in_open_list(pp_next) == False:
            ListProperty.o.append(pp_next)

    def judge_est_distance(self,pp):
        """用于比较总距离，更新开表内节点"""

        ori_distance = self.get_est_whole_distrance(pp)#得到原先总距离

        cur_distance = pp.g + 1 + self.get_estimate_distance(pp)#获取当前距离
        #原先总距离和当前距离相比较
        if ori_distance <= cur_distance:
            pass
        else:
            #查找该点在列表中的索引位置
            cur_index = ListProperty.o.index(pp)
            ListProperty.o[cur_index].g = cur_distance
        pass

    def is_passable(self,*location):
        """判断此点是否为可通行点"""
        # 带星号的形参传过来的是元组
        if self.map_matrix[self.pp.y + location[0][1]][self.pp.x+location[0][0]] == "#":
            return False
        else:
            return True

    def in_open_list(self,pp):
        """判断某点是否处于开启列表中"""
        r_value = False
        for i in range(0,len(ListProperty.o)):
            if pp.x == ListProperty.o[i].x and pp.y == ListProperty.o[i].y:
                r_value = True
                break
        return r_value

    def in_close_list(self,*location):
        """判断某点是否处于已关闭列表中"""
        pp = PointProperty()
        pp.x = self.pp.x + location[0][0]
        pp.y = self.pp.y + location[0][1]
        r_value = False
        for i in range(0,len(ListProperty.c)):
            if ListProperty.c[i].x == pp.x and ListProperty.c[i].y == pp.y:
                r_value = True
                break
        return r_value

    def write_the_route(self):
        """将规划好的路径写入到map_matrix中"""
        def search_point_o_and_c(x,y):
            """返回在开表和闭表里搜索的点对象"""
            for i in range(0,len(ListProperty.c)):
                if ListProperty.c[i].x == x and ListProperty.c[i].y == y:
                    return ListProperty.c[i]
            for i in range(0,len(ListProperty.o)):
                if ListProperty.o[i].x == x and ListProperty.o[i].y == y:
                    return ListProperty.o[i]

        while not (self.pp.x == self.sta_x and self.pp.y == self.sta_y):
            ListProperty.route.append((self.pp.x, self.pp.y))
            # 根据父节点的坐标在开表和闭表中寻找父节点
            bx = self.pp.bx
            by = self.pp.by
            self.pp = search_point_o_and_c(bx,by)

            print "route is ",ListProperty.route
        ListProperty.route.append((self.pp.x,self.pp.y))

    def output_the_route(self):
        """输出规划好了的路径矩阵"""
        for (i,j) in ListProperty.route:
            if self.map_matrix[j][i] == ".":
                self.map_matrix[j][i] = "*"
        Pre_Handle.output_map(self.map_matrix)


if __name__ =="__main__":
    ra = RouteAlgorithm()
    ra.main()
    pass
