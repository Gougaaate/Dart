import numpy as np

def get_variable_name(x):
    return [i for i, a in locals().items() if a == x][0]

class Log:
    dt = .1 # par défaut
    var_names = []
    current_data = []   

    def __init__(self):
        Log.f = open("log.txt", "w")

    def add_to_current_data(**data):
        if Log.var_names is not None:
            Log.var_names += [key + " " for key in data.keys()]
        #print(Log.var_names)

        Log.current_data += list(data.values())

    def write_current_data():
        if Log.var_names is not None:
            Log.f.write("".join(Log.var_names) + "\n")
            Log.var_names = None

        Log.f.write("".join([str(d)+" " for d in Log.current_data]) + "\n")

    def plot_data():
        with open("log.txt", "r") as f:
            vars = f.readline().split(" ")[:-1]
            print(vars)
            n = int(input("Nombre de figures : "))
            plots = []
            for j in range(n):
                plots.append([int(i) for i in input("Quels plots sur la figure " + str(j+1) + "? (ex: 0 1 3)").split(" ")])

            data = []
            for line in f.readlines():
                data.append([float(coef) for coef in line.split(" ")[:-1]])
            data = np.array(data)

            t = np.linspace(0, Log.dt*len(data), len(data))
            for i in range(n):
                plt.figure(i+1)
                plt.title(vars[i])
                for j in range(len(plots[i])):
                    plt.plot(t, data[:, plots[i][j]])
            plt.show()

    def __del__(self):
        Log.f.close()

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    Log.plot_data()
