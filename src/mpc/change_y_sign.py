import pickle

with open("/home/piotr/catkin_ws/src/selfie_f1tenth2019/mpc/map.pkl", 'rb') as f:
    map_data = pickle.load(f)
pathpoints = map_data['pathpoints']

data = {}
data['pathpoints'] = [
    (-1*pathpoints[i][1] -3.5, pathpoints[i][0] +3.5)
    for i in range(len(pathpoints))
]

with open('mapChanged.pkl', 'wb') as fx:
    pickle.dump(data, fx, 2)
