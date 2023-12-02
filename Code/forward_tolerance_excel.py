import math
import csv
from csv import reader

lb =  205    #distance from origin to motor attachment point in mm
le =  50     #distance from centre of moving platform to link attachment point in mm
rf =  400    #Upper link length in mm  - F1J1, F2J2, F3J3 as in the paper
re =  1000   #Lower link length in mm  - JIE1, J2E2, J3E3 as in the paper

def forward_tolerance(theta1,theta2,theta3, rf1, rf2, rf3, re1, re2, re3) :   # thetas are in radians

    theta1 = math.radians(theta1)
    theta2 = math.radians(theta2)
    theta3 = math.radians(theta3)

    x1 = 0
    y1 = (-1)*(lb-le)-(rf1)*(math.cos(theta1))
    z1 = (-1)*(rf1)*(math.sin(theta1))

    x2 = (math.sqrt(3))*(lb-le+(rf2)*(math.cos(theta2)))/2.0
    y2 = (lb-le+(rf2)*(math.cos(theta2)))/2.0
    z2 = (-1)*(rf2)*(math.sin(theta2))

    x3 = (math.sqrt(3))*(-lb+le-(rf3)*(math.cos(theta3)))/2.0
    y3 = (lb-le+(rf3)*(math.cos(theta3)))/2.0
    z3 = (-1)*(rf3)*(math.sin(theta3))

    w1 = x1**2 + y1**2 + z1**2
    w2 = x2**2 + y2**2 + z2**2
    w3 = x3**2 + y3**2 + z3**2

    # y = a2(z) + b2
    d2 = ((x3)*(y1-y2)-(x2)*(y1-y3))
    a2 = ((x2)*(z1-z3)-(x3)*(z1-z2))/d2
    b2 = (((-1)*(x2)*(w1-w3+re3**2-re1**2)+(x3)*(w1-w2+re2**2-re1**2))/2.0)/d2

    #x = a1(z) + b1
    d1 = ((x3)*(y2-y3)+(x2-x3)*(y1-y3))
    a1 = ((y2-y3)*(z1-z3)-(y1-y3)*(z2-z3))/d1
    b1 = ((y1-y3)*(w2-w3+re3**2-re2**2)*(0.5)-(y2-y3)*(w1-w3+re3**2-re1**2)*(0.5))/d1

    # (a1^2 +a2^2 +1)z^2 + (2a1b1 +2a2b2 -2a2y1 -2z1)z + (-re^2 +y1^2 +z1^2 -2b2y1 +b2^2 +b1^2) = 0
    a = a1**2 + a2**2 + 1
    b = 2*(a1)*(b1) + 2*(a2)*(b2) - 2*(a2)*(y1) - 2*(z1)
    c = (-1)*(re1**2) + y1**2 + z1**2 - 2*(b2)*(y1) + b2**2 + b1**2

    D = b**2 - 4*(a)*(c)

    if D<0 :
        print('Non-existent point\n')
        return 0

    else :
        z = (-b - math.sqrt(D))/(2*a)
        y = (a2)*(z) + b2
        x = (a1)*(z) + b1
        #Modification to match paper's Coordinate system but not flipping the z axis
        x,y = (-1)*y,x

        array = (x,y,z)
        return array

x = 0.01 #10 microns in mm
tolerance_values = [-x,x,(-2)*x,2*x,(-3)*x,3*x,(-4)*x,4*x,(-5)*x,5*x,(-6)*x,6*x,(-7)*x,7*x,(-8)*x,8*x,(-9)*x,9*x,(-10)*x,10*x]
maxerrors = []
correspondingvalues = []
holdervalues = []

with open('forward_tolerance.csv', mode='w') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['Theta1','Theta2','Theta3','x1','y1','z1','MaxError1','x2','y2','z2','MaxError2','x3','y3','z3','MaxError3','x4','y4','z4','MaxError4','x5','y5','z5','MaxError5','x6','y6','z6','MaxError6','x7','y7','z7','MaxError7','x8','y8','z8','MaxError8','x9','y9','z9','MaxError9','x10','y10','z10','MaxError10'])

with open('forward_kinematics.csv', mode = 'r') as read_obj :
    csv_reader = reader(read_obj)
    header = next(csv_reader)
    for row in csv_reader :
        theta1 = float(row[0])
        theta2 = float(row[1])
        theta3 = float(row[2])
        x0 = float(row[3])
        y0 = float(row[4])
        z0 = float(row[5])
        maxerror = 0
        maxerrors = []
        correspondingvalues = []
        holdervalues = []
        i = 0
        for i in (0,2,4,6,8,10,12,14,16,18):
            t1 = tolerance_values[i]
            t2 = tolerance_values[i+1]
            maxerror = 0
            for L11 in (0, t1, t2) :
                for L12 in (0, t1, t2) :
                    for L21 in (0, t1, t2) :
                        for L22 in (0, t1, t2) :
                            for L31 in (0, t1, t2) :
                                for L32 in (0, t1, t2) :
                                    holder = forward_tolerance(theta1,theta2,theta3,rf+L11,rf+L21,rf+L31,re+L12,re+L22,re+L32)
                                    newerror = math.sqrt((x0 - holder[0])**2 + (y0 - holder[1])**2 + (z0 - holder[2])**2)
                                    if newerror > maxerror:
                                        maxerror = newerror
                                        correspondingvalue = (rf+L11,rf+L21,rf+L31,re+L12,re+L22,re+L32)
                                        maxcoordinates = holder
            maxerrors.append(maxerror)
            correspondingvalues.append(correspondingvalue)
            holdervalues.append(maxcoordinates)
        with open('forward_tolerance.csv', mode='a') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([theta1,theta2,theta3,holdervalues[0][0],holdervalues[0][1],holdervalues[0][2],maxerrors[0],holdervalues[1][0],holdervalues[1][1],holdervalues[1][2],maxerrors[1],holdervalues[2][0],holdervalues[2][1],holdervalues[2][2],maxerrors[2],holdervalues[3][0],holdervalues[3][1],holdervalues[3][2],maxerrors[3],holdervalues[4][0],holdervalues[4][1],holdervalues[4][2],maxerrors[4],holdervalues[5][0],holdervalues[5][1],holdervalues[5][2],maxerrors[5],holdervalues[6][0],holdervalues[6][1],holdervalues[6][2],maxerrors[6],holdervalues[7][0],holdervalues[7][1],holdervalues[7][2],maxerrors[7],holdervalues[8][0],holdervalues[8][1],holdervalues[8][2],maxerrors[8],holdervalues[9][0],holdervalues[9][1],holdervalues[9][2],maxerrors[9]])