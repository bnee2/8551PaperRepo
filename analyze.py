from os import walk
from json import load

# Loops over all files in a directory and prints stats about their performance
def main():
    paths = [
        # "/home/brandon/Documents/CSCI8551/Project/Paper/WHCAData",
        "/home/brandon/Documents/CSCI8551/Project/Paper/ICTSData",
        # "/home/brandon/Documents/CSCI8551/Project/Paper/CBSData"
    ]
    for path in paths:
        print("\n\nStart OF " + path)
        for root, dirs, files in walk(path):
            for file in sorted(files):
                if "warehouse" in file:
                    continue
                print(file)
                jsonFile = open(path + "/" + file)
                jsonData = load(jsonFile)
                numAgents = int(jsonData["teamSize"])
                numTimeSteps = len(jsonData["plannerTimes"])
                averagePlanningTime = round(sum([float(time) for time in jsonData["plannerTimes"]]) / numTimeSteps, 3)
                tasksPerStep = round(float(jsonData["numTaskFinished"]) / numTimeSteps, 3)
                tasksPerStepPerAgent = round(tasksPerStep / numAgents, 3)
                print("Tasks Completed: " + str(jsonData["numTaskFinished"]))
                print("Average Planning Time: " + str(averagePlanningTime))
                print("Tasks Per Step: " + str(tasksPerStep))
                print("Tasks Per Step Per Agent: " + str(tasksPerStepPerAgent))
                print()
                jsonFile.close()
        print("END OF " + path + "\n\n")

if __name__ == "__main__":
    main()