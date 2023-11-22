from os import scandir
from json import load
from random import randint

WIDTH = 32
HEIGHT = 32
FILLRATE = 15
AGENTS = 50
TASKS = 10000

# Loops over all files in a directory and prints stats about their performance
def main():
    validStarts = []
    with open("mapData.map", "w") as file:
        file.write("type octile\n")
        file.write("height " + str(HEIGHT) + "\n")
        file.write("width " + str(WIDTH) + "\n")
        file.write("map\n")
        for y in range(HEIGHT):
            string = ""
            for x in range(WIDTH):
                if randint(1,100) < FILLRATE:
                    string += "@"
                else:
                    string += "."
                    validStarts.append(y*HEIGHT + x)
            file.write(string + "\n")

    starts = []
    with open("agentData.agents", "w") as file:
        file.write(str(AGENTS) + "\n")
        while len(starts) < AGENTS:
            index = randint(0,len(validStarts)-1)
            if validStarts[index] not in starts:
                starts.append(validStarts[index])
        for start in starts:
            file.write(str(start) + "\n")

    taskIndices = []
    with open("taskData.tasks", "w") as file:
        file.write(str(TASKS) + "\n")
        while len(taskIndices) < TASKS:
            index = randint(0,len(validStarts)-1)
            taskIndices.append(validStarts[index])
        for taskIndex in taskIndices:
            file.write(str(taskIndex) + "\n")


if __name__ == "__main__":
    main()