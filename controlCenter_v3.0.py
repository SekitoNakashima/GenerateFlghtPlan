import os
import json
import math
import copy
import pandas as pd
import meshID
import aStarAlgo as aStar
import two_point_distance as tpd

# 飛行計画の書き込み
def generateFlightPlan(name, plan):

    # JSON読み取り
    jsonFile = open('./flightPlan/{name}'.format(name=name), 'r')
    fp = json.load(jsonFile)

    planLen = len(plan)               # ウェイポイントの数
    st = plan[0][1]                   # スタート時間
    gt = plan[planLen-1][2]           # ゴール時間
    interval = (gt-st) / (planLen+1)  # ウェイポイント間の時間

    flightPointArray = []

    for i in range(-1, planLen+1):
        # スタート
        if i < 0:
            time = st
            wID = -1
        # ゴール
        elif i == planLen:
            time = gt
            wID = -1
        else:
            time = st + ((i+1)*interval)
            wID = plan[i][0]

        point = {
            'time': time,
            'wayPointID': wID
        }
        flightPointArray.append(point)

    fp['flightPointArray'] = flightPointArray

    # JSON書き込み
    newFp = open('./newFlightPlan/{name}'.format(name=name), 'w')
    json.dump(fp, newFp, indent=4)

def generateFlightPlan2(name, plan):
    # JSON読み取り
    jsonFile = open('./flightPlan/{name}'.format(name=name), 'r')
    fp = json.load(jsonFile)
    
    flightPointArray = []
    for i in range(1, 3):
        point = {
            'time': plan[0][i],
            'wayPointID': -1
        }
        flightPointArray.append(point)

    fp['flightPointArray'] = flightPointArray

    # JSON書き込み
    newFp = open('./newFlightPlan/{name}'.format(name=name), 'w')
    json.dump(fp, newFp, indent=4)

# 衝突判定
def collisionJudge(timeList, sTime2, gTime2):
    #print(timeList)
    for time in timeList:
        sTime1 = time[0]
        gTime1 = time[1]

        if (sTime1 <= sTime2 and sTime2 <= gTime1) or (sTime1 <= gTime2 and gTime2 <= gTime1):
            return True
    return False

# 希望の位置, 時間を取得
def getCondition(name):
    with open('./flightPlan/{name}'.format(name=name), "r") as f:
        fp = json.load(f)
    
    startLat = fp["startRequest"]["position"]["latitude"]
    startLon = fp["startRequest"]["position"]["longitude"]
    goalLat = fp["goalRequest"]["position"]["latitude"]
    goalLon = fp["goalRequest"]["position"]["longitude"]
    startTime = fp["startRequest"]["time"]
    goalTime = fp["goalRequest"]["time"]
    
    return startLat, startLon, goalLat, goalLon, startTime, goalTime


# 飛行計画の数を取得
directory = os.listdir("flightPlan") # directoryに全ての飛行計画名を格納
ngList = []                          # NGリスト
listMesh = {}                        # 作成済みのタイムポイントメッシュ

# メッシュ決定
#mesh = pd.read_csv("./mesh/meshSize100.csv")
meshSize = 50

# 経路探索
route = aStar.routeSearch(meshSize)

# メッシュID
mesh = meshID.mesh(meshSize)

# 飛行計画の数だけ探索
for name in directory:
    print(name)
    routeDic = {}      # 探索済みの経路
    distanceDic = {}   # 探索済み経路の距離
    route_search_num = 0
    searchState = False  # 探索済みかどうか
    addTime1 = 0  # Start 遅らせる時間
    addTime2 = 0  # Goal  遅らせる時間

    # 希望のスタートとゴール取得
    startLat, startLon, goalLat, goalLon, startTime, goalTime = getCondition(name)

    # スタートとゴールのメッシュの行列番号を取得
    sx, sy = mesh.pointToMeshID(startLat, startLon)
    gx, gy = mesh.pointToMeshID(goalLat, goalLon)

    while True :  # 衝突を回避するまでループ
        collision = False  # 衝突判定のフラッグ

        # スタートとゴールの時間調整
        startTime += addTime1
        goalTime += addTime2
        print("r: ", route_search_num)

        # 経路探索
        if searchState == True:
            plan = copy.deepcopy(routeDic[str(route_search_num)][0])
            distance = distanceDic[str(route_search_num)][0]
        else:
            plan, distance = route.main(sx, sy, gx, gy, ngList)

        # 運行時間を遅らせる(経路探索で経路が見つからなかったとき, 速度違反(200m/s)のとき)
        if len(plan) == 0 or (distance/(goalTime-startTime)) > 200:
            if route_search_num == 0:
                addTime1 = 0
                addTime2 = math.ceil(distance/200)  # スタート時間+addTime2で理想のゴール時間
            else:
                addTime1 = 3
                addTime2 = 3
                searchState = True
                routeDic[str(route_search_num)] = []
                distanceDic[str(route_search_num)] = []
                routeDic[str(route_search_num)].append([])
                distanceDic[str(route_search_num)].append(distance)
            route_search_num = 0
            ngList = []
            continue

        # 探索した経路は保存
        if searchState == False:
            tmp = copy.deepcopy(plan)
            routeDic[str(route_search_num)] = []
            distanceDic[str(route_search_num)] = []

            routeDic[str(route_search_num)].append(tmp)
            distanceDic[str(route_search_num)].append(distance)

        # 経路探索回数を更新
        route_search_num += 1

        # タイムポイントメッシュ化
        if distance < 0:  # スタートとゴールのメッシュが同じ
            tmp = [[plan[0][0], startTime, goalTime]]
        else:
            tmp = mesh.toTimePointMesh(plan, startTime, goalTime)
        newPlan = copy.deepcopy(tmp)

        # 衝突判定
        for k in range(len(newPlan)):
            if str(newPlan[k][0]) in listMesh: # 空間的衝突判定
                if collisionJudge(listMesh[str(newPlan[k][0])], newPlan[k][1], newPlan[k][2]):# 時間的衝突判定
                    ngList.append(newPlan[k][0])
                    collision = True
                    break  # 衝突が起こる
    
        if collision == False:  # 衝突は起こらない
            # ハッシュ更新
            for j in range(len(newPlan)):
                if not str(newPlan[j][0]) in listMesh: # キーが無かったら初期化 以降追記
                    listMesh[str(newPlan[j][0])] = []
                listMesh[str(newPlan[j][0])].append([newPlan[j][1], newPlan[j][2]])

            # 飛行計画をJSON形式で生成
            if distance < 0: # スタートとゴールが重なる
                generateFlightPlan2(name, newPlan)
            else:
                generateFlightPlan(name, newPlan)

            break
    ngList = []
    routeDic = {}
    distanceDic = {}