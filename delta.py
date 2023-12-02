import math
import csv
from csv import reader

e = 50*(math.sqrt(3)) #end-effector equilateral triangle side length
f = 205*(math.sqrt(3)) #fixed base equilateral triangle side length
re = 1000 #lower-arm length
rf = 400 #upper-arm legth
min = int(math.degrees((-1)*(2/9)*(math.pi)))
max = int(math.degrees((4/9)*(math.pi)))
step = int(math.degrees(math.pi/180.0))

# forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
# returned status: 0=OK, -1=non-existing position
def delta_calcForward(theta1, theta2, theta3):

    x0 = 0
    y0 = 0
    z0 = 0

    t = (f-e)*math.tan(math.radians(30))/2.0

    #degrees to radians
    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)

    y1 = -(t + rf*math.cos(theta1))
    z1 = -rf*math.sin(theta1)

    y2 = (t + rf*math.cos(theta2))*math.sin(math.radians(30))
    x2 = y2*math.tan(math.radians(60))
    z2 = -rf*math.sin(theta2)

    y3 = (t + rf*math.cos(theta3))*math.sin(math.radians(30))
    x3 = -y3*math.tan(math.radians(60))
    z3 = -rf*math.sin(theta3)

    dnm = (y2-y1)*x3-(y3-y1)*x2

    w1 = y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2

    # x = (a1*z + b1)/dnm
    a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
    b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0

    # y = (a2*z + b2)/dnm
    a2 = -(z2-z1)*x3+(z3-z1)*x2
    b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0

    # a*z^2 + b*z + c = 0
    a = a1**2 + a2**2 + dnm**2
    b = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm**2)
    c = (b2-y1*dnm)**2 + b1**2 + dnm**2*(z1**2 - re**2)

    # discriminant
    d = b**2 - 4.0*a*c;
    if (d < 0):
        return -1; # non-existing point

    z0 = -0.5*(b+math.sqrt(d))/a
    x0 = (a1*z0 + b1)/dnm
    y0 = (a2*z0 + b2)/dnm
    return (x0, y0, z0)

    # inverse kinematics
    # helper functions, calculates angle theta1 (for YZ-pane)
def delta_calcAngleYZ(x0,y0,z0, theta1):
    y1 = -0.5 * 0.57735 * f # f/2 * tg 30
    y0 -= 0.5 * 0.57735 * e # shift center to edge
    # z = a + b*y
    a = (x0**2 + y0**2 + z0**2 + rf**2 - re**2 - y1**2)/(2*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a+b*y1)*(a+b*y1)+rf*(b**2*rf+rf)
    if (d < 0):
        return -1; # non-existing point
    yj = (y1 - a*b - math.sqrt(d))/(b**2 + 1) # choosing outer point
    zj = a + b*yj
    if yj > y1 :
        temp = 180.0
    else :
        temp = 0.0
    theta1 = 180.0*math.atan(-zj/(y1 - yj))/pi + temp
    return (0, theta1)

    # inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
    # returned status: 0=OK, -1=non-existing position
def delta_calcInverse(x0, y0, z0):
    theta1 = theta2 = theta3 = 0
    status[0] = delta_calcAngleYZ(x0, y0, z0, theta1)[0]
    status[1] = delta_calcAngleYZ(x0, y0, z0, theta1)[1]
    if (status[0] == 0): #calculation for theta2
        status[2] = delta_calcAngleYZ(x0*math.cos(math.radians(120)) + y0*math.sin(math.radians(120)), y0*math.cos(math.radians(120))-x0*math.sin(math.radians(120)), z0, theta2)  # rotate coords to +120 deg
    if (status[0] == 0): #calculation for theta3
        status[3] = delta_calcAngleYZ(x0*math.cos(math.radians(120)) - y0*math.sin(math.radians(120)), y0*math.cos(math.radians(120))+x0*math.sin(math.radians(120)), z0, theta3) # rotate coords to -120 deg
    return status

forward_columns = ['Theta1', 'Theta2', 'Theta3', 'x0', 'y0', 'z0']
with open('forward_kinematics.csv', mode='w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(forward_columns)
for theta1 in range(min,max,step):
    for theta2 in range(min,max,step):
        for theta3 in range(min,max,step):
            holder = delta_calcForward(theta1, theta2, theta3)
            with open('forward_kinematics.csv', mode='a') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([theta1, theta2, theta3, holder[0], holder[1], holder[2]])