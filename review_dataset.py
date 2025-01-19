import pickle

with open('dataset.txt', 'r') as f:
    lines = f.read().splitlines()
ticks_list = []
# Imprime las primeras líneas del archivo para revisar su formato
for l in lines[9:]:
    tokens = l.split(":")
    ticks = tokens[2].strip()
    ticks = ticks.split(" ")
    #print(float(ticks[0]), float(ticks[1]))
    #print([float(ticks[0]), float(ticks[1])])
    ticks_list.append([float(ticks[0]), float(ticks[1])])
#print(ticks_list)
ticks_trans = [[ticks_list[i][0]-ticks_list[i-1][0], ticks_list[i][1]-ticks_list[i-1][1]] for i in range(1, len(ticks_list))]

"""with open('dataset/ticks_transitions.pkl', 'wb') as f:
    pickle.dump(ticks_trans, f)"""