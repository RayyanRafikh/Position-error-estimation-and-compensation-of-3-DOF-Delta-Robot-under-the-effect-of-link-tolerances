import math

lb =  205    #distance from origin to motor attachment point in mm
le =  50     #distance from centre of moving platform to link attachment point in mm
rf =  400    #Upper link length in mm
re =  1000   #Lower link length in mm

print('Enter the coordinates of the center of moving platform in millimeters')
x0 = float(input('X : '))
y0 = float(input('Y : '))
z0 = float(input('Z : '))
#modifications to match the coordinate system of the paper but not flipping the z axis
x0,y0 = y0,(-1)*x0

def delta_calcAngleYZ(x0, y0, z0) :
    y1 = -lb
    y0 -= le
    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0)
    b = (y1-y0)/z0
    # discriminant
    d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf)
    if (d < 0) :
        return -1 # non-existing point
    yj = (y1 - a*b - math.sqrt(d))/(b*b + 1) # choosing outer point
    zj = a + b*yj
    if yj > y1:
         x = 180.0
    else :
         x = 0.0
    theta = 180.0*math.atan(-zj/(y1 - yj))/math.pi + x
    theta = math.radians(theta)
    status = (0,theta)
    return status

 # inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 # returned status: 0=OK, -1=non-existing position
def inverse_kinematics(x0, y0, z0):
    status = delta_calcAngleYZ(x0, y0, z0)
    if (status[0] == 0):
        theta2 = delta_calcAngleYZ(x0*math.cos(math.radians(120)) + y0*math.sin(math.radians(120)), y0*math.cos(math.radians(120))-x0*math.sin(math.radians(120)), z0)[1]  # rotate coords to +120 deg
    if (status[0] == 0):
        theta3 = delta_calcAngleYZ(x0*math.cos(math.radians(120)) - y0*math.sin(math.radians(120)), y0*math.cos(math.radians(120))+x0*math.sin(math.radians(120)), z0)[1] # rotate coords to -120 deg
    h = (status[1], theta2, theta3)
    return h

h = inverse_kinematics(x0,y0,z0)
print(h)
print(math.degrees(h[0]),math.degrees(h[1]),math.degrees(h[2]))