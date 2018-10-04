import os
import csv
import operator

source = '/home/nick/Desktop/Robotics/References/Propeller/UIUC Prop Data/UIUC-propDB/volume-1/data'

thrustData = []
csvf = open('data.csv', 'w')
wf = csv.writer(csvf)
propData = dict()

for root, dirs, readFilenames in os.walk(source):
    # Iterate through all files in the given directory
    for readFile in readFilenames:
        # Grab the prop diameter and pitch
        splitReadFile = readFile.split("_")

        print splitReadFile

        prop = splitReadFile[1].split("x")
        name = splitReadFile[0]

        hashVal = (name, prop[0], prop[1])

        # Locate the readFile
        location = source + "/" + readFile
        
        # rf is the read file
        rf = open(location, 'r')

        # The first line orf each .txt is the header
        title = rf.readline().split(" ")
        newTitle = []
        for item in title:
            if (item != ''):
                newTitle.append(item)

        # Stationary Data, J = 0
        if (newTitle[0] == 'RPM'):            
            pass
            """
            J = 0
            data = rf.readline()

            while (data):
                splitData = data.split(' ')

                newData = []
                for datum in splitData:
                    if (datum != ''):
                        newData.append(datum)

                newData[2] = newData[2].split('\n')[0]

                data = rf.readline()

            thrustData = [name, prop[0], prop[1], newData[0], J, newData[1], 
                newData[2], 0]
            
            if (propData.has_key(hashVal)):
                item = propData[hashVal]
            else:
                item = []

            item.append(thrustData)
            propData[hashVal] = item
            """

        # Variable Advancement Ratio
        elif (newTitle[0] == 'J'):
            RPM = splitReadFile[3].split('.txt')[0]
            data = rf.readline()

            while (data):
                splitData = data.split(' ')

                newData = []
                for datum in splitData:
                    if (datum != ''):
                        newData.append(datum)

                newData[3] = newData[3].split('\n')[0]

                thrustData = [name, prop[0], prop[1], RPM, newData[0], newData[1], 
                    newData[2], newData[3]]
                
                if (propData.has_key(hashVal)):
                    item = propData[hashVal]
                else:
                    item = []

                item.append(thrustData)
                propData[hashVal] = item

                data = rf.readline()

    for key in propData.keys():
        data = propData[key]
        data.sort(key = operator.itemgetter(4))

        for datum in data:
            wf.writerow(datum)

