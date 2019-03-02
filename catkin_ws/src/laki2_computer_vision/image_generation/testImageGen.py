import random
import string
import os
import numpy as np
import cv2
import math
import io
from PIL import Image, ImageDraw, ImageColor, ImageFont


def main():
    generateImage(400, 5, 10, "Double Dab")


def generateImage(imSize, blurAmount=0, noiseProb=0, filename="dab", forceLetterColor=None, forceBackColor=None, forceShapeColor=None):

    shapes = ["Circle", "Square", "Rectangle", "Semi", "Quarter",
              "Triangle", "Pentagon", "Hexagon", "Cross", "Trapezoid",
              "Heptagon", "Octogon"]
    colors = [("Red", (255, 0, 0, 255)), ("Green", (0, 255, 0, 255)), ("Blue", (0, 0, 255, 255)),
              ("Yellow", (255, 255, 0, 255)), ("Purple", (255, 0, 255, 255)),
              ("Cyan", (0, 255, 255, 255)), ("Black", (0, 0, 0, 255)), ("White", (255, 255, 255, 255))]

    shapeChoice = random.choice(shapes)
    shapeColorChoice = random.choice(colors)
    letterChoice = random.choice(string.ascii_uppercase)
    letterColorChoice = shapeColorChoice

    backgroundColor = colors[len(colors) - 1]

    while letterColorChoice == shapeColorChoice:
        letterColorChoice = random.choice(colors)

    if not forceLetterColor == None:
        for c in colors:
            if c[0] == forceLetterColor:
                letterColorChoice = c

    if not forceShapeColor == None:
        for c in colors:
            if c[0] == forceShapeColor:
                shapeColorChoice = c

    if not forceBackColor == None:
        for c in colors:
            if c[0] == forceBackColor:
                backgroundColor = c

    filePath = os.path.dirname(os.path.abspath(__file__))

    if not os.path.isdir(os.path.join(filePath, "generatedImages")):

        try:
            os.makedirs(os.path.join(filePath, "generatedImages"))
        except OSError:
            print("Cannot Create Path")
        else:
            print("Directory created")

    newImage = None
    
    if  forceBackColor == None:
        newImage = Image.new('RGBA', (imSize, imSize), (random.randrange(
            0, 100), random.randrange(200, 255), random.randrange(0, 100), 255))
    else:
        newImage = Image.new('RGBA', (imSize, imSize), backgroundColor[1])

    imageCenter = (imSize/2, imSize/2)
    objectCenter = (random.randint(100, 300), random.randint(100, 300))

    drawIm = ImageDraw.Draw(newImage)

    angle = random.randrange(0, 359)
    angle = math.radians(angle)

    if shapeChoice == "Square":
        points = list([(80, -80), (80, 80), (-80, 80), (-80, -80)])

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Rectangle":
        points = list([(80, -50), (80, 50), (-80, 50), (-80, -50)])

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Triangle":
        points = equalShape(3, 80)

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Pentagon":
        points = equalShape(5, 80)

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Hexagon":
        points = equalShape(3, 80)

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Heptagon":
        points = equalShape(7, 80)

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Octogon":
        points = equalShape(7, 80)

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Trapezoid":
        points = list([(-40, 45), (40, 45), (80, -45), (-80, -45)])

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Cross":
        points = list([(-80, -27), (-27, -27), (-27, -80), (26, -80),
                       (26, - 27), (80, - 27), (80, 26), (26, 26),
                       (26, 80), (-27, 80), (-27, 26), (-80, 26)])

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Circle":
        points = equalShape(32, 80)

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])

    if shapeChoice == "Quarter":
        points = equalShape(16, 160, (math.pi/2), (80, 80))
        points.append((-80,-80))
        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        
        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])
                         
    if shapeChoice == "Semi":
        points = equalShape(24, 80, math.pi, (0,40))

        for i in range(0, len(points)):
            points[i] = rotatePoint(points[i], angle)
            points[i] = (points[i][0] + objectCenter[0],
                         points[i][1] + objectCenter[1])

        
        drawIm.polygon(points, shapeColorChoice[1], shapeColorChoice[1])
    
    
    pasteLetter = drawLetter(letterChoice, letterColorChoice, filePath)

    recPoint1 = (-50, -50)
    recPoint2 = (50,50)

    pasteLetter = pasteLetter.rotate(math.degrees(angle), center=(50,50))
    recPoint1 = rotatePoint(recPoint1, angle)
    recPoint2 = rotatePoint(recPoint2, angle)

    newImage.paste(pasteLetter, (int(objectCenter[0]) - 50, int(objectCenter[1]) - 50), pasteLetter)
    
    newImage.save("generatedImages/" + filename + ".png")
    newImage = cv2.imread("generatedImages/" + filename + ".png")
    if noiseProb > 0:
        newImage = noise(newImage, noiseProb)
    if blurAmount > 0:
        newImage = cv2.blur(newImage, (random.randint(
            1, blurAmount), random.randint(1, blurAmount)))
    cv2.imwrite("generatedImages/" + filename + ".png", newImage)

    fileToWrite = open("generatedImages/" + filename + ".txt", "w")

    fileToWrite.write("Shape: " + shapeChoice + "\n")
    fileToWrite.write("ShapeColor: " + shapeColorChoice[0] + "\n")
    fileToWrite.write("ShapePosition: " + str(objectCenter) + "\n")
    fileToWrite.write("Letter: " + letterChoice + "\n")
    fileToWrite.write("LetterColor: " + letterColorChoice[0] + "\n")
        

    fileToWrite.close()


def noise(image, amount):
    output = np.zeros(image.shape, np.uint8)
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            noiAmount = random.uniform(-amount, amount)
            r = clamp(image[i][j][0] + noiAmount, 0, 255)
            g = clamp(image[i][j][1] + noiAmount, 0, 255)
            b = clamp(image[i][j][2] + noiAmount, 0, 255)
            output[i][j] = (r, g, b)

    return output

def drawLetter(letter, letterColor, filePath):
    im = Image.new('RGBA', (100, 100))

    dim = ImageDraw.Draw(im)

    font = ImageFont.truetype(filePath + "/American Captain.ttf", 90)

    w, h = dim.textsize(letter)
    w *= 7
    h *= 7
    x = 50 - (w / 2)
    y = 50 - (h / 2)
    print (w, h)

    # dim.text(((100 - w)/2,(100-h)/2), str(letter), letterColor[1], font)
    dim.text((x, y), str(letter), letterColor[1], font)
    
    return im
    




def clamp(val, rangemin, rangemax):
    if val < rangemin:
        val = rangemin
    if val > rangemax:
        val = rangemax
    return val


def rotatePoint(point, angle, center=(0, 0)):
        return (math.cos(angle) * (point[0] - center[0]) - math.sin(angle) * (point[1] - center[1]), math.sin(angle) * (point[0] - center[0]) + math.cos(angle) * (point[1]) - center[1])


def equalShape(numOfSides, distanceFromCenter, maxAngle=(2 * math.pi), offset=(0, 0)):
    points = list()
    for i in range(0, numOfSides + (0 if maxAngle == (2 * math.pi) else 1)):
        points.append((distanceFromCenter * math.cos(i/numOfSides * maxAngle) - offset[0],
                       distanceFromCenter * math.sin(i/numOfSides * maxAngle) - offset[1]))
    return points


if __name__ == '__main__':
    main()
