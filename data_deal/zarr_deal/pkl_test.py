import pickle

path = "/home/wmx/myspace/RDP/data/test1/seq_0001.pkl"
with open(path, "rb") as f:
    data = pickle.load(f)

print(type(data))
print(hasattr(data, "sensorMessages"))
print(len(data.sensorMessages))