# Importing Libraries
import math
import random

import serial
import time

import pygame

pygame.init()

# Set the width and height of the screen [width, height]
size = (700, 500)
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF , 32)

pygame.display.set_caption("Car Controller V1.8")

arduinoConnected = True


def setSpeed(x):
    packet = bytearray()
    packet.append(123)
    packet.append(111)
    packet.append(3)  # length
    packet.append(0)  # set
    packet.append(50)  # RW speed
    # packet.append(23) #increment value
    packet.append(int(x))
    arduino.write(packet)


def getRWNibble():
    packet = bytearray()
    packet.append(123)
    packet.append(111)
    packet.append(2)  # length
    packet.append(1)  # get
    packet.append(2)  # right speed
    arduino.write(packet)
    #time.sleep(0.05)
    # and now read it
    pkttype = ord(arduino.read())  # type
    pktlen = ord(arduino.read())  # msgID
    valueret = ord(arduino.read())
    print("Type = ", pkttype, ". Len = ", pktlen, ". RW nibble: ", valueret, "\n")


def getRWCurrent():
    packet = bytearray()
    packet.append(123)
    packet.append(111)
    packet.append(2)  # length
    packet.append(1)  # get
    packet.append(0)  # right speed
    arduino.write(packet)
    pkttype = ord(arduino.read())  # type
    pktlen = ord(arduino.read())  # msgID
    valueret = ord(arduino.read());
    valueret = valueret + ord(arduino.read()) * 256;
    print("Type = ", pkttype, ". Len = ", pktlen, ". RW Current: ", valueret, "\n")


pressRight = False
pressLeft = False
pressUp = False
pressDown = False


def writeText(string, coordx, coordy, fontSize, colour, center=True, background=None):
    # set the font to write with
    font = pygame.font.SysFont('calibri', fontSize)
    if background == None:
        # (0, 0, 0) is black, to make black text
        text = font.render(string, True, colour)
    else:
        text = font.render(string, True, colour, background)
    # get the rect of the text
    textRect = text.get_rect()
    # set the position of the text
    if center:
        textRect.center = (coordx, coordy)
    else:
        textRect.topleft = (coordx, coordy)
    screen.blit(text, textRect)


def drawMenuArrows(screen, centerX, centerY):
    # Polygon center is at 150, 150, thus:
    centerX -= 150
    centerY -= 150
    if pressUp:
        pygame.draw.polygon(screen, (255, 25, 25), [(100 + centerX, 100 + centerY),
                                                    (200 + centerX, 100 + centerY),
                                                    (150 + centerX, 44 + centerY)])
    else:
        pygame.draw.polygon(screen, (25, 25, 25), [(100 + centerX, 100 + centerY),
                                                   (200 + centerX, 100 + centerY),
                                                   (150 + centerX, 44 + centerY)])

    if pressDown:
        pygame.draw.polygon(screen, (255, 25, 25), [(100 + centerX, 200 + centerY),
                                                    (200 + centerX, 200 + centerY),
                                                    (150 + centerX, 256 + centerY)])
    else:
        pygame.draw.polygon(screen, (25, 25, 25), [(100 + centerX, 200 + centerY),
                                                   (200 + centerX, 200 + centerY),
                                                   (150 + centerX, 256 + centerY)])

    if pressLeft:
        pygame.draw.polygon(screen, (255, 25, 25), [(100 + centerX, 100 + centerY),
                                                    (100 + centerX, 200 + centerY),
                                                    (44 + centerX, 150 + centerY)])
    else:
        pygame.draw.polygon(screen, (25, 25, 25), [(100 + centerX, 100 + centerY),
                                                   (100 + centerX, 200 + centerY),
                                                   (44 + centerX, 150 + centerY)])

    if pressRight:
        pygame.draw.polygon(screen, (255, 25, 25), [(200 + centerX, 100 + centerY), (200 + centerX, 200 + centerY),
                                                    (256 + centerX, 150 + centerY)])
    else:
        pygame.draw.polygon(screen, (25, 25, 25), [(200 + centerX, 100 + centerY), (200 + centerY, 200 + centerY),
                                                   (256 + centerX, 150 + centerY)])


def drawProximityBars(screen, height, width, sideDistance):
    size = screen.get_size()
    pygame.draw.rect(screen, (25, 25, 25), (sideDistance, (size[1] - height) / 2, width, height), border_radius=12)
    pygame.draw.rect(screen, (255 * (1 - Car.leftWheelRange), 255 * Car.leftWheelRange, 20), (
        sideDistance, (size[1] - height) / 2 + height * (1 - Car.leftWheelRange), width, height * Car.leftWheelRange),
                     border_radius=12)
    writeText(str(round(Car.leftWheelRange * 100, 2)) + "cm", sideDistance + width / 2 + 30, (size[1] - height) / 2 - 10 , 36,
              (255, 255, 255))

    pygame.draw.rect(screen, (25, 25, 25), (size[0] - sideDistance - width, (size[1] - height) / 2, width, height),
                     border_radius=12)
    pygame.draw.rect(screen, (255 * (1 - Car.rightWheelRange), 255 * Car.rightWheelRange, 20), (
        size[0] - sideDistance - width, (size[1] - height) / 2 + height * (1 - Car.rightWheelRange), width,
        height * Car.rightWheelRange), border_radius=12)
    writeText(str(round(Car.rightWheelRange * 100, 2)) + "cm", size[0] - sideDistance - width / 2 - 30, (size[1] - height) / 2 - 10,
              36, (255, 255, 255))


sideValue = 0
forwardValue = 0.5


def drawInstructionBars(screen, height, width, sideDistance):
    size = screen.get_size()
    pygame.draw.rect(screen, (25, 25, 25), ((size[0] - width) / 2, size[1] - height - sideDistance, width, height),
                     border_radius=12)
    if sideValue > 0:
        pygame.draw.rect(screen, (25, 255, 255),
                         (size[0] / 2, size[1] - height - sideDistance, width * 0.5 * sideValue, height),
                         border_bottom_right_radius=12, border_top_right_radius=12)

    else:
        pygame.draw.rect(screen, (0, 255, 255),
                         (size[0] / 2 + width * 0.5 * sideValue, size[1] - height - sideDistance,
                          -width * 0.5 * sideValue, height), border_top_left_radius=12, border_bottom_left_radius=12)

    pygame.draw.rect(screen, (25, 25, 25), (350 - height / 2, 100, height, 125), border_radius=12)

    pygame.draw.rect(screen, (205 * (1 - forwardValue) + 50, 205 * forwardValue + 50, 50),
                     (350 - height / 2, 100 + 125 * (1 - forwardValue), height, 125 * forwardValue), border_radius=12)


zAngle = 0
viewAngle = 30


class sonarLoop():
    def __init__(self, center):
        self.age = 0
        self.decay = 0.1
        self.center = center

    def tick(self):
        self.age += self.decay

    def draw(self, screen):
        pygame.draw.ellipse(screen, (125 - 0.8 * self.age, 125 - 0.8 * self.age, 125 - 0.8 * self.age), (
            self.center[0] - self.age * math.cos(math.radians(viewAngle)),
            self.center[1] - self.age * math.sin(math.radians(viewAngle)),
            2 * self.age * math.cos(math.radians(viewAngle)),
            2 * self.age * math.sin(math.radians(viewAngle))), 2)


sonarLoops = []
frameCounter = 0


def drawDirectionLines(screen, center, length):
    global frameCounter

    pygame.draw.ellipse(screen, (25, 25, 25), (
        center[0] - length * math.cos(math.radians(viewAngle)), center[1] - length * math.sin(math.radians(viewAngle)),
        length * 2 * math.cos(math.radians(viewAngle)) + 2, 2 * length * math.sin(math.radians(viewAngle)) + 2))

    for i in range(0, 360, 360//8):
        pygame.draw.line(screen, (50, 50, 50), center, (
        center[0] - math.cos(math.radians(viewAngle)) * length * math.sin(math.radians(i)),
        center[1] - math.sin(math.radians(viewAngle)) * length * math.cos(math.radians(i))))

    pygame.draw.line(screen, (12, 12, 125), center,
                     (center[0], center[1] + length * 0.5 * math.cos(math.radians(viewAngle))))

    frameCounter += 1
    if frameCounter > 200:
        frameCounter = 0
        sonarLoops.append(sonarLoop(center))

    removeLoops = []
    for i, loop in enumerate(sonarLoops):
        loop.tick()
        if loop.age > 120:
            removeLoops.append(loop)
        loop.draw(screen)

    for loop in removeLoops:
        sonarLoops.remove(loop)
    pygame.draw.line(screen, (255, 25, 25), center, (
        center[0] - math.cos(math.radians(viewAngle)) * length * math.sin(math.radians(zAngle)),
        center[1] - math.sin(math.radians(viewAngle)) * length * math.cos(math.radians(zAngle))))
    # pygame.draw.ellipse(screen, (255, 25, 25), ((center[0] - math.cos(math.radians(viewAngle)) * length * math.sin(math.radians(zAngle)) - 3 * math.cos(math.radians(viewAngle)) , center[1] - math.sin(math.radians(zAngle)) * length * math.cos(math.radians(viewAngle)) - 3 * math.sin(math.radians(viewAngle)), 6 * math.cos(math.radians(viewAngle)), 6 * math.sin(math.radians(viewAngle)))))
    pygame.draw.line(screen, (25, 255, 25), (
        center[0] + math.cos(math.radians(viewAngle)) * length * 0.5 * math.sin(math.radians(zAngle) - math.pi / 2),
        center[1] + math.sin(math.radians(viewAngle)) * length * 0.5 * math.cos(math.radians(zAngle) - math.pi / 2)), (
        center[0] - math.cos(math.radians(viewAngle)) * length * 0.5 * math.sin(math.radians(zAngle) - math.pi / 2),
        center[1] - math.sin(math.radians(viewAngle)) * length * 0.5 * math.cos(math.radians(zAngle) - math.pi / 2)))
    pygame.draw.line(screen, (25, 25, 255), center, (center[0], center[1] - length * 0.5 * math.cos(math.radians(viewAngle))))

    points = []
    l1 = length  * Car.leftWheelRange
    for i in range(15, 46):
        points.append((center[0] - math.cos(math.radians(viewAngle)) * l1 * math.sin(math.radians(zAngle + i)),
                       center[1] - math.sin(math.radians(viewAngle)) * l1 * math.cos(math.radians(zAngle + i))))
    for i in range(45, 15, -1):
        points.append((center[0] - math.cos(math.radians(viewAngle)) * l1 * math.sin(math.radians(zAngle + i)),
                       center[1] - math.sin(math.radians(viewAngle)) * l1 * math.cos(math.radians(zAngle + i)) - 10))

    pygame.draw.polygon(screen, (255 * (1 - Car.leftWheelRange), 255 * (Car.leftWheelRange), 20), points)

    points = []
    l2 = length  * Car.rightWheelRange
    for i in range(-45, -14):
        points.append((center[0] - math.cos(math.radians(viewAngle)) * l2 * math.sin(math.radians(zAngle + i)),
                       center[1] - math.sin(math.radians(viewAngle)) * l2 * math.cos(math.radians(zAngle + i))))
    for i in range(-15, -46, -1):
        points.append((center[0] - math.cos(math.radians(viewAngle)) * l2 * math.sin(math.radians(zAngle + i)),
                       center[1] - math.sin(math.radians(viewAngle)) * l2 * math.cos(math.radians(zAngle + i)) - 10))

    pygame.draw.polygon(screen, (255 * (1 - Car.rightWheelRange), 255 * (Car.rightWheelRange), 20), points)


wheelOffset = 0


def drawWheelSim(screen, center, length, width):
    colour = (50, 50, 50)
    pygame.draw.circle(screen, colour, center, length, width=width)
    spokes = 8
    for i in range(0, 360, 360 // spokes):
        pygame.draw.line(screen, colour, center, (center[0] + length * math.sin(math.radians(i + wheelOffset)) * .9,
                                                  center[1] + length * math.cos(math.radians(i + wheelOffset)) * .9),
                         width=3)
    pygame.draw.line(screen, (200, 200, 200), center, (center[0] + length * math.sin(math.radians(wheelOffset)) * .95,
                                                       center[1] + length * math.cos(math.radians(wheelOffset)) * .95),
                     width=3)

    pygame.draw.rect(screen, (25, 25, 25), (center[0] + length * 1.2, center[1] - length, 10, length * 2),
                     border_radius=12)
    pygame.draw.rect(screen, (50, 50, 50), (center[0] + length * 1.2, center[1] - length + (1-Car.rightWheelCurrent/100) * length * 2, 10, Car.rightWheelCurrent/100 * length * 2), border_radius=12)
    writeText(str(int(Car.rightWheelCurrent / 100 * 250)) + "mA", center[0] + length * 1.2, center[1] - length - 5, 36,
              (100, 100, 100))

    pygame.draw.rect(screen, (25, 25, 25), (center[0] - length * 1.2 - 10, center[1] - length, 10, length * 2),
                     border_radius=12)
    pygame.draw.rect(screen, (50, 50, 50), (center[0] - length * 1.2 - 10, center[1] - length + (1-Car.leftWheelCurrent/100) * length * 2, 10, Car.leftWheelCurrent/100 * length * 2), border_radius=12)
    writeText(str(int(Car.leftWheelCurrent/100 * 250)) + "mA", center[0] - length * 1.2 - 10, center[1] - length - 5, 36, (100, 100, 100))


prevPositions = []


def drawSurfacePlot(screen, margin, height):
    size = screen.get_size()
    pygame.draw.rect(screen, (25, 25, 25), (margin, margin, size[0] / 2 - margin, height))
    for i, pos in enumerate(prevPositions):
        pygame.draw.line(screen, (255, 25, 25), (i - 1 + margin, prevPositions[i - 1]), (i + margin, pos))
        # pygame.draw.circle(screen, (25, 255, 25), (margin + i, margin + height - pos), 1)

def drawBatteryLevel(screen):
    writeText(str(round(Car.batteryLevel /100 * 2.2 + 5, 2)) + "V", 16, screen.get_size()[1] - 48, 40, (25, 255, 255), center=False)

def drawPOVbackground(screen):
    for i in range(-90, 90, 15):
        pygame.draw.line(screen, (25, 25, 25), (screen.get_size()[0]/2, screen.get_size()[1] * 4), (screen.get_size()[0]/2 + 400 * math.sin(math.radians(i)), screen.get_size()[1]/2))
    for i in range(0, 90, 10):
        pygame.draw.line(screen, (25, 25, 25), (0, (math.sin(math.radians(i)) + 1) * screen.get_size()[1]/2) , (screen.get_size()[0], (math.sin(math.radians(i)) + 1) * screen.get_size()[1]/2))
def drawCarBody(screen):
    pygame.draw.ellipse(screen, (50, 50, 50), (screen.get_size()[0] * .1, screen.get_size()[1] * .9, screen.get_size()[0] * .8, screen.get_size()[1] * .2))

def drawData(screen):
    data = Car.print()


    writeText("RW Cur: " + str(round(data["RW Curr"] , 2)) + "mA", 40, 10 + 0 * 50, 48, (255, 255, 255), center=False)
    writeText("RW Rng: " + str(round(data["RW Rng"] * 100, 2)) + "cm", 40, 10 + 1 * 50, 48, (255, 255, 255), center=False)
    writeText("RW Nib: " + str(round(data["RW Nib"], 2)), 40, 10 + 2 * 50, 48, (255, 255, 255), center=False)
    writeText("LW Cur: " + str(round(data["LW Cur"] * 20, 2)) + "mA", 40, 10 + 3 * 50, 48, (255, 255, 255), center=False)
    writeText("LW Rng: " + str(round(data["LW Rng"] * 100, 2)) + "cm", 40, 10 + 4 * 50, 48, (255, 255, 255), center=False)
    writeText("LW Nib: " + str(round(data["LW Nib"], 2)), 40, 10 + 5 * 50, 48, (255, 255, 255), center=False)
    writeText("Bat: " + str(round(data["Battery"] / 100 * 2.2 + 5, 2)) + "V", 40, 10 + 6 * 50, 48, (255, 255, 255), center=False)

    if sideValue + 0.05 < 0:
        writeText("Turning left", 40, 10 + 8 * 50, 48, (255, 255, 255), center=False)
    elif sideValue - 0.05 > 0:
        writeText("Turning right", 40, 10 + 8 * 50, 48, (255, 255, 255), center=False)


menuMode = "HUD"
def drawMenu(screen):
    if menuMode == "HUD":
        ######### Draw Arrows ###########

        drawMenuArrows(screen, 350, 350)

        ######### Draw Proximity Bars ###########

        drawProximityBars(screen, 350, 20, 20)
        drawInstructionBars(screen, 20, 350, 20)

        ######### Draw Direction Lines ##########

        drawDirectionLines(screen, (200, 175), 120)

        ######## Draw Wheel Sim ############

        drawWheelSim(screen, (500, 175), 75, 12)

        ########## Draw Surface Plot ##########

        drawSurfacePlot(screen, 10, 40)

        ########## Draw Battery Level ###########

        drawBatteryLevel(screen)
    elif menuMode == "DATA":
        ######### Draw Arrows ###########

        #drawMenuArrows(screen, 350, 350)

        ######### Draw Proximity Bars ###########

        #drawInstructionBars(screen, 20, 350, 20)

        ########## Draw Battery Level ###########

        #drawBatteryLevel(screen)

        drawData(screen)

class car():
    def __init__(self):
        self.rightWheelCurrent = 0
        self.rightWheelRange = 0
        self.rightWheelNibble = 0

        self.leftWheelCurrent = 0
        self.leftWheelRange = 0
        self.leftWheelNibble = 0

        self.batteryLevel = 0

        self.accelerometerX = 0
        self.accelerometerY = 0

    def populateSelf(self, values):
        self.rightWheelCurrent = values[0] / 33
        self.rightWheelRange = values[1] / 3300
        self.rightWheelNibble = values[2]

        self.leftWheelCurrent = values[3] / 33
        self.leftWheelRange = values[4] / 1000
        if self.leftWheelRange > 1:
            self.leftWheelRange = 1
        self.leftWheelNibble = values[5]

        self.batteryLevel = values[6] // 33

        self.accelerometerX = values[7]
        self.accelerometerY = values[8]
        self.print()

    def print(self):
        print("\r" + str({
            "RW Curr": self.rightWheelCurrent,
            "RW Rng": self.rightWheelRange,
            "RW Nib": self.rightWheelNibble,

            "LW Cur": self.leftWheelCurrent,
            "LW Rng": self.leftWheelRange,
            "LW Nib": self.leftWheelNibble,

            "Battery": self.batteryLevel,

            "Acl X": self.accelerometerX,
            "Acl Y": self.accelerometerY
        }))
        return {
            "RW Curr": self.rightWheelCurrent,
            "RW Rng": self.rightWheelRange,
            "RW Nib": self.rightWheelNibble,

            "LW Cur": self.leftWheelCurrent,
            "LW Rng": self.leftWheelRange,
            "LW Nib": self.leftWheelNibble,

            "Battery": self.batteryLevel,

            "Acl X": self.accelerometerX,
            "Acl Y": self.accelerometerY
        }


Car = car()


class protocolHandler():
    def __init__(self):
        self.cmd = {
            "get": 1,
            "set": 0
        }

        self.idCMD = {
            "rightWheelCurrent": 0,
            "rightRange": 1,
            "rightWheelNibble": 2,

            "leftWheelCurrent": 11,
            "leftRange": 12,
            "leftWheelNibble": 13,

            "batteryVoltage": 20,
            "accelerometer": 30,

            "everything": 40,

            "rightWheelSpeed": 50,
            "leftWheelSpeed": 51,

        }

        self.lengthCMD = {
            "rightWheelCurrent": 2,
            "rightWheelRange": 2,
            "rightWheelNibble": 1,

            "everything": 18,
        }

    def setWheelSpeed(self):
        packet = bytearray()
        packet.append(123)
        packet.append(111)
        packet.append(3)
        packet.append(0)
        packet.append(52)
        packet.append(int(forwardValue * 100))
        arduino.write(packet)

    def setDirectionWheelSpeed(self, left=None, right=None):
        packet = bytearray()
        packet.append(123)
        packet.append(111)
        packet.append(3)
        packet.append(0)
        packet.append(50)
        if left is None and right is None:
            if sideValue <= 0:
                leftVal = forwardValue * 75
            else:
                leftVal = forwardValue * 75 * (1 - sideValue)
        else:
            leftVal = forwardValue * 100 * left
            print("Left Val: " + str(leftVal))
        packet.append(int(leftVal))
        arduino.write(packet)

        packet = bytearray()
        packet.append(123)
        packet.append(111)
        packet.append(3)
        packet.append(0)
        packet.append(51)
        if left is None and right is None:
            if sideValue >= 0:
                rightVal = forwardValue * 75
            else:
                rightVal = forwardValue * 75 * (1 + sideValue)
        else:

            rightVal = forwardValue * 100 * right
            print("Right Val: " + str(rightVal))

        packet.append(int(rightVal))
        arduino.write(packet)


    def getEverything(self):
        packet = bytearray()
        packet.append(123)
        packet.append(111)
        packet.append(2)
        packet.append(1)
        packet.append(40)
        arduino.write(packet)
        #time.sleep(0.05)
        # and now read it
        pkttype = ord(arduino.read())  # type
        pktlen = ord(arduino.read())  # msgID

        valueRWCurr = ord(arduino.read()) + ord(arduino.read()) * 256
        valueRWRange = ord(arduino.read()) + ord(arduino.read()) * 256
        valueRWNibble = ord(arduino.read())

        valueLWCurr = ord(arduino.read()) + ord(arduino.read()) * 256
        valueLWRange = ord(arduino.read()) + ord(arduino.read()) * 256
        valueLWNibble = ord(arduino.read())

        valueBatteryLvl = ord(arduino.read()) + ord(arduino.read()) * 256
        valueAccelX = ord(arduino.read()) + ord(arduino.read()) * 256
        valueAccelY = ord(arduino.read()) + ord(arduino.read()) * 256

        arduino.read()
        arduino.read()

        Car.populateSelf(
            [valueRWCurr, valueRWRange, valueRWNibble, valueLWCurr, valueLWRange, valueLWNibble, valueBatteryLvl,
             valueAccelX, valueAccelY])


if arduinoConnected:
    handlerBT = protocolHandler()
    arduino = serial.Serial(port='COM17', baudrate=115200, timeout=0.5)
# print(arduino)


choice = 0
updateTime = time.time()
prevForwardVal = forwardValue
prevSideVal = sideValue
if __name__ == "__main__":
    done = False
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    done = True
                if event.key == pygame.K_LEFT:
                    pressLeft = True
                if event.key == pygame.K_RIGHT:
                    pressRight = True
                if event.key == pygame.K_UP:
                    pressUp = True
                if event.key == pygame.K_DOWN:
                    pressDown = True
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT:
                    pressLeft = False
                if event.key == pygame.K_RIGHT:
                    pressRight = False
                if event.key == pygame.K_UP:
                    pressUp = False
                if event.key == pygame.K_DOWN:
                    pressDown = False

        if pressRight and sideValue < 1:
            sideValue += 0.005
        if pressLeft and sideValue > -1:
            sideValue -= 0.005

        zAngle -= sideValue

        if prevSideVal < sideValue - 0.1 or prevSideVal > sideValue + 0.1:
            prevSideVal = sideValue
            handlerBT.setDirectionWheelSpeed()

        if pressUp and forwardValue < 1:
            forwardValue += 0.005
        if pressDown and forwardValue > 0:
            forwardValue -= 0.005

        if prevForwardVal < forwardValue - 0.05 or prevForwardVal > forwardValue + 0.05:
            prevForwardVal = forwardValue

            if Car.rightWheelCurrent < 20 or Car.leftWheelCurrent < 20:
                handlerBT.setWheelSpeed()
            elif Car.rightWheelCurrent > Car.leftWheelCurrent:
                handlerBT.setDirectionWheelSpeed(left=1, right=Car.leftWheelCurrent/Car.rightWheelCurrent)
            elif Car.rightWheelCurrent < Car.leftWheelCurrent:
                handlerBT.setDirectionWheelSpeed(left=Car.rightWheelCurrent/Car.leftWheelCurrent, right=1)

        wheelOffset += 0.5 * forwardValue

        if not pressLeft and not pressRight:
            if sideValue > 0:
                sideValue -= 0.015
            if sideValue < 0:
                sideValue += 0.015
            if -0.005 < sideValue < 0.005:
                sideValue = 0

        prevPositions.append(random.randint(19, 20))
        if len(prevPositions) > screen.get_size()[0] / 2 - 10:
            prevPositions.pop(0)
            # --- Game logic should go here

            # --- Screen-clearing code goes here

            # Here, we clear the screen to white. Don't put other drawing commands
            # above this, or they will be erased with this command.

            # If you want a background image, replace this clear with blit'ing the
            # background image.
        screen.fill((0, 0, 0))

        if updateTime < time.time() - 0.25:
            updateTime = time.time()
            handlerBT.getEverything()

        drawMenu(screen)

        # --- Drawing code should go here

        # --- Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

    # Close the window and quit.
    pygame.quit()

    # print(arduino.read())
    arduino.close()
