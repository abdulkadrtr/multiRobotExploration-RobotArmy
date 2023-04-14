import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import heapq , math , time , threading
import scipy.interpolate as si
import datetime

lookahead_distance = 0.22 #one bakma mesafesi
speed = 0.18 #maksimum hiz
expansion_size = 4 #duvar genisletme katsayisi
target_error = 0.15 #hedefe olan hata payi

TB0_PATH = [(0,0)]
TB0_PATHF = 0
TB1_PATH = [(0,0)]
TB1_PATHF = 0
VISITED = []

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i-1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:
                    matrix[i][j] = 2
    return matrix

def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups

def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups) # sağ alt çapraz
    dfs(matrix, i - 1, j - 1, group, groups) # sol üst çapraz
    dfs(matrix, i - 1, j + 1, group, groups) # sağ üst çapraz
    dfs(matrix, i + 1, j - 1, group, groups) # sol alt çapraz
    return group + 1

def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid

def visitedControl(targetP):
    global VISITED
    for i in range(len(VISITED)):
        k = VISITED[i]
        d = math.sqrt((k[0] - targetP[0])**2 + (k[1] - targetP[1])**2)
        if d < 0.2:
            return 1
    return 0

def findClosestGroup(matrix,groups, current,resolution,originX,originY,choice):
    global TB0_PATH
    global TB1_PATH
    global TB0_PATHF
    global TB1_PATHF
    targetP = None
    distances = []
    paths = []
    lengths = []
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        t = (middle[1]*resolution+originX,middle[0]*resolution+originY)
        if visitedControl(t) == 0:
            path = astar(matrix, current, middle)
            path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
            total_distance = pathLength(path)
            distances.append(total_distance) #ROTA UZUNLUĞU
            paths.append(path) #ROTA
            lengths.append(len(groups[i][1])) #SINIR ÇİZGİSİ NOKTA SAYISI
    #ROTA | ROTA UZUNLUĞU | SINIR ÇİZGİSİ NOKTA SAYISI
    arrays = list(zip(lengths,distances,paths))
    arrays = sorted(arrays, key=lambda x: x[1], reverse=False) #ROTA UZUNLUĞUNA GÖRE SIRALAMA
    #Rota uzunluğu target_error*3 ten küçük olanlar çıkarılır.
    arrays_a = [a for a in arrays if a[1] > target_error*2]
    p1 = [a for a in arrays_a if a[1] < 2.0] #Rota uzunluğu 2.0 dan küçük olanlar seçilir.
    #p1 elemanlarından sınır çizgisi nokta sayısı en büyük olan seçilir.
    p1 = sorted(p1, key=lambda x: x[0], reverse=True)
    #p1 boş değilse seçilir.
    if len(p1) > 0:
        targetP = p1[0][2]
    else:
        p1 = [a for a in arrays_a if a[1] < 4.0]
        p1 = sorted(p1, key=lambda x: x[0], reverse=True)
        if len(p1) > 0:
            targetP = p1[0][2]
    if targetP == None:
        arrays_a = sorted(arrays, key=lambda x: x[0], reverse=True)
        targetP = arrays_a[0][2]
    if targetP == None:
        #Burada lengths değeri en büyük olan seçilir.
        p1 = [a for a in arrays if a[0] == max(lengths)]
        if len(p1) > 0:
            targetP = p1[0][2]
    return targetP

def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def exploration(data,width,height,resolution,column,row,originX,originY,choice):
    global TB0_PATH
    global TB1_PATH
    global TB0_PATHF
    global TB1_PATHF
    f = 1
    data = costmap(data,width,height,resolution) #Engelleri genislet
    data[row][column] = 0 #Robot Anlık Konum
    data[data > 5] = 1 # 0 olanlar gidilebilir yer, 100 olanlar kesin engel
    data = frontierB(data) #Sınır noktaları bul
    data,groups = assign_groups(data) #Sınır noktaları gruplandır
    groups = fGroups(groups) #Grupları küçükten büyüğe sırala. En buyuk 5 grubu al
    if len(groups) == 0: #Grup yoksa kesif tamamlandı
        f = -1
    else: #Grup varsa en yakın grubu bul
        data[data < 0] = 1 #-0.05 olanlar bilinmeyen yer. Gidilemez olarak isaretle. 0 = gidilebilir, 1 = gidilemez.
        path = findClosestGroup(data,groups,(row,column),resolution,originX,originY,choice) #En yakın grubu bul
        if path != None: #Yol varsa BSpline ile düzelt
            path = bspline_planning(path,len(path)*5)
        else:
            f = -1
    if choice == 0:
        if f == -1:
            TB0_PATHF = f
        else:
            TB0_PATH = path
            TB0_PATHF = f
    elif choice == 1:
        if f == -1:
            TB1_PATHF = f
        else:
            TB1_PATH = path
            TB1_PATHF = f
    return

def get(choice):
    now = datetime.datetime.now()
    time_string = now.strftime("%H:%M:%S")
    print(f"[BILGI] {time_string}: {choice+1}. ROBOT TARAFINDAN YOL ISTEGI ALINDI")

def response(choice):
    now = datetime.datetime.now()
    time_string = now.strftime("%H:%M:%S")
    print(f"[BILGI] {time_string}: {choice+1}. ROBOTA YOL ISTEK CEVABI GONDERILDI")

class HeadquartersControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription = self.create_subscription(OccupancyGrid,'merge_map',self.map_callback,10)
        self.publisher_tb3_0_path = self.create_publisher(Float32MultiArray,'tb3_0/path', 10)
        self.publisher_tb3_1_path = self.create_publisher(Float32MultiArray,'tb3_1/path', 10)
        self.subscription_tb3_0_odom = self.create_subscription(Odometry,'tb3_0/odom',self.tb0_odom_callback,10)
        self.subscription_tb3_1_odom = self.create_subscription(Odometry,'tb3_1/odom',self.tb1_odom_callback,10)
        self.subscription_tb3_0_cmd_vel = self.create_subscription(Twist,'tb3_0/cmd_vel',self.tb0_status_control,4)
        self.subscription_tb3_1_cmd_vel = self.create_subscription(Twist,'tb3_1/cmd_vel',self.tb1_status_control,4)
        self.publiser_drone_loc = self.create_publisher(Pose,'drone/target_pose',10)
        print("[BILGI] KESIF MODU AKTIF")
        print("[BILGI] DRONE GOZLEM MODU AKTIF")
        self.kesif = True
        threading.Thread(target=self.start_exploration_r0).start() #Kesif fonksiyonunu thread olarak calistirir. Robot1
        threading.Thread(target=self.start_exploration_r1).start() #Kesif fonksiyonunu thread olarak calistirir. Robot2
        threading.Thread(target=self.start_exploration_r2).start() #Kesif fonksiyonunu thread olarak calistirir. Robot3


        

    def start_exploration_r0(self):
        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'tb0_x'):
                continue
            self.c_tb0 = int((self.tb0_x - self.originX)/self.resolution)
            self.r_tb0 = int((self.tb0_y - self.originY)/self.resolution)
            if self.kesif:
                if TB0_PATHF == 0:
                    get(0)
                    exploration(self.data,self.width,self.height,self.resolution,self.c_tb0,self.r_tb0,self.originX,self.originY,0)
                    response(0)
                    self.tb0_path_pub()
                    self.tb0_s = False
                if TB0_PATHF == -1:
                    self.kesif = False
                    print("[BILGI] KESIF TAMAMLANDI")
                    print("[BILGI] KESIF MODU PASIF")
                    break
                time.sleep(0.1)
                if self.tb0_s == True:
                    get(0)
                    self.t0.join()
                    response(0)
                    self.tb0_path_pub()
                time.sleep(0.4)

    def start_exploration_r1(self):
        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'tb1_x'):
                continue
            self.c_tb1 = int((self.tb1_x - self.originX)/self.resolution)
            self.r_tb1 = int((self.tb1_y - self.originY)/self.resolution)
            if self.kesif:
                if TB1_PATHF == 0:
                    get(1)
                    exploration(self.data,self.width,self.height,self.resolution,self.c_tb1,self.r_tb1,self.originX,self.originY,1)
                    response(1)
                    self.tb1_path_pub()
                    self.tb1_s = False
                if TB1_PATHF == -1:
                    self.kesif = False
                    print("[BILGI] KESIF TAMAMLANDI")
                    print("[BILGI] KESIF MODU PASIF")
                    break
                time.sleep(0.1)
                if self.tb1_s == True:
                    get(1)
                    self.t1.join()
                    response(1)
                    self.tb1_path_pub()
                time.sleep(0.4)
                                   
    def start_exploration_r2(self):
        while True:
            if not hasattr(self, 'tb0_x') or not hasattr(self, 'tb1_x'):
                continue
            print("[BILGI] DRONE GOZLEM NOKTASI HEDEFI GONDERILDI")
            time.sleep(5)
            pose = Pose()
            x = (self.tb0_x + self.tb1_x)/2
            y = (self.tb0_y + self.tb1_y)/2
            z = math.sqrt((self.tb0_x - self.tb1_x)**2 + (self.tb0_y - self.tb1_y)**2) + 6
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            self.publiser_drone_loc.publish(pose)
            time.sleep(15)

    def target_callback(self,msg):
        if msg == 0:
            c = int((TB0_PATH[-1][0] - self.originX)/self.resolution)
            r = int((TB0_PATH[-1][1] - self.originY)/self.resolution)
        elif msg == 1:
            c = int((TB1_PATH[-1][0] - self.originX)/self.resolution)
            r = int((TB1_PATH[-1][1] - self.originY)/self.resolution)
        exploration(self.data,self.width,self.height,self.resolution,c,r,self.originX,self.originY,msg)

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

    def tb0_odom_callback(self,msg):
        self.tb0_x = msg.pose.pose.position.x
        self.tb0_y = msg.pose.pose.position.y

    def tb1_odom_callback(self,msg):
        self.tb1_x = msg.pose.pose.position.x
        self.tb1_y = msg.pose.pose.position.y  

    def tb0_status_control(self,msg):
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.tb0_s = True
        else:
            self.tb0_s = False #False ise robot hareket ediyor. MESGUL
    
    def tb1_status_control(self,msg):
        if msg.linear.x == 0 and msg.angular.z == 0:
            self.tb1_s = True
        else:
            self.tb1_s = False #False ise robot hareket ediyor. MESGUL

    def tb0_path_pub(self):
        global TB0_PATH
        global VISITED
        VISITED.append(TB0_PATH[-1])
        message = Float32MultiArray()
        message.data = [elem for tup in TB0_PATH for elem in tup]
        self.publisher_tb3_0_path.publish(message)
        t = pathLength(TB0_PATH)/speed
        t = t - 0.2
        if t < 0:
            t = 0
        self.t0 = threading.Timer(t, self.target_callback,args=(0,))
        self.t0.start()
    
    def tb1_path_pub(self):
        global TB1_PATH
        global VISITED
        VISITED.append(TB1_PATH[-1])
        message = Float32MultiArray()
        message.data = [elem for tup in TB1_PATH for elem in tup]
        self.publisher_tb3_1_path.publish(message)
        t = pathLength(TB1_PATH)/speed
        t = t - 0.2
        if t < 0:
            t = 0
        self.t1 = threading.Timer(t, self.target_callback,args=(1,))
        self.t1.start()


def main(args=None):
    rclpy.init(args=args)
    headquarters_control = HeadquartersControl()
    rclpy.spin(headquarters_control)
    headquarters_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
