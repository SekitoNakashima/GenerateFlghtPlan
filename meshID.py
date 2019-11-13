class mesh:
    def __init__(self, meshSize):
        self.originLat = 34.6
        self.originLon = 133.9

        self.meshNum = 5000 / meshSize
        self.latSize = 0.04491574308 / self.meshNum
        self.lonSize = 0.05483191182 / self.meshNum
        

    def pointToMeshID(self, lat, lon):
        
        # 座標をメッシュIDに
        x = int((lat - self.originLat) % self.latSize)
        y = int((lon - self.originLon) // self.lonSize)

        return x, y

    def toTimePointMesh(self, plan, stime, gtime):
        # 時間間隔(元のスタートとゴールを含める)
        interval = (gtime - stime) / (len(plan)+1)

        # スタート, ゴールのタイムポイントメッシュ
        endInterval = interval / 2
        plan[0].append(stime)
        plan[0].append(stime+endInterval+interval)
        plan[len(plan)-1].append(gtime-endInterval-interval)
        plan[len(plan)-1].append(gtime)

        for i in range(1, len(plan)-1):
            plan[i].append(stime+endInterval+(i*interval))
            plan[i].append(stime+endInterval+((i+1)*interval))
        #print(plan)
        return plan