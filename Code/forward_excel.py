import math
import csv
from csv import reader

lb =  205    #distance from origin to motor attachment point in mm
le =  50     #distance from centre of moving platform to link attachment point in mm
rf =  400    #Upper link length in mm  - F1J1, F2J2, F3J3 as in the paper
re =  1000   #Lower link length in mm  - JIE1, J2E2, J3E3 as in the paper

min = int(math.degrees((-1)*(2/9)*(math.pi)))
max = int(math.degrees((4/9)*(math.pi)))
step = int(math.degrees((4)*(math.pi/180.0)))

def forward_kinematics(theta1,theta2,theta3) :   # thetas are in radians

    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)

    x1 = 0
    y1 = (-1)*(lb-le)-(rf)*(math.cos(theta1))
    z1 = (-1)*(rf)*(math.sin(theta1))

    x2 = (math.sqrt(3))*(lb-le+(rf)*(math.cos(theta2)))/2.0
    y2 = (lb-le+(rf)*(math.cos(theta2)))/2.0
    z2 = (-1)*(rf)*(math.sin(theta2))

    x3 = (math.sqrt(3))*(-lb+le-(rf)*(math.cos(theta3)))/2.0
    y3 = (lb-le+(rf)*(math.cos(theta3)))/2.0
    z3 = (-1)*(rf)*(math.sin(theta3))

    w1 = x1**2 + y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2

    # y = a2(z) + b2
    d2 = ((x3)*(y1-y2)-(x2)*(y1-y3))
    a2 = ((x2)*(z1-z3)-(x3)*(z1-z2))/d2
    b2 = (((-1)*(x2)*(w1-w3)+(x3)*(w1-w2))/2.0)/d2

    #x = a1(z) + b1
    d1 = ((x3)*(y2-y3)+(x2-x3)*(y1-y3))
    a1 = ((y2-y3)*(z1-z3)-(y1-y3)*(z2-z3))/d1
    b1 = ((y1-y3)*(w2-w3)*(0.5)-(y2-y3)*(w1-w3)*(0.5))/d1

    # (a1^2 +a2^2 +1)z^2 + (2a1b1 +2a2b2 -2a2y1 -2z1)z + (-re^2 +y1^2 +z1^2 -2b2y1 +b2^2 +b1^2) = 0
    a = a1**2 + a2**2 + 1
    b = 2*(a1)*(b1) + 2*(a2)*(b2) - 2*(a2)*(y1) - 2*(z1)
    c = (-1)*(re**2) + y1**2 + z1**2 - 2*(b2)*(y1) + b2**2 + b1**2

    D = b**2 - 4*(a)*(c)

    if D<0 :
        print('Non-existent point\n')
        return 0

    else :
        z = (-b - math.sqrt(D))/(2*a)
        y = (a2)*(z) + b2
        x = (a1)*(z) + b1
        #Modification to match paper's Coordinate system but not flipping the z axis
        x,y= (-1)*y,x

        array = (x,y,z)
        return array

forward_columns = ['Theta1', 'Theta2', 'Theta3', 'x0', 'y0', 'z0']

with open('forward_kinematics.csv', mode='w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(forward_columns)

for theta1 in range(min,max+1,step):
    for theta2 in range(min,max+1,step):
        for theta3 in range(min,max+1,step):
            holder = forward_kinematics(theta1,theta2,theta3)
            with open('forward_kinematics.csv', mode='a') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([theta1, theta2, theta3, holder[0], holder[1], holder[2]])
