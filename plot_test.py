import os
import json
import os
import matplotlib.pyplot as plt

def plotter(name):
    
    with open(name, "r") as file:
        data = json.load(file)

    cur_time=[]
    delivery_ratio=[]

    for seed in data["10"]:
        cur_time.append(seed[0])

    for seed in data.keys():
        delivery_ratio = []
        for y in data[seed]:
            if y[0] == 0:
                delivery_ratio.append(0)
            else:
                delivery_ratio.append(y[1])
        plt.plot(cur_time, delivery_ratio, label="Seed "+str(seed))


    plt.xticks(cur_time,rotation="vertical")
    plt.xlabel('Step)')
    plt.ylabel('Delivery Ratio(%)')
    plt.legend()
    plt.show()


plotter('DroNETworkSimulator-hmw1/test_30_AI.json')