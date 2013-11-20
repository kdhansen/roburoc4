import math
import numpy


def xyz2gps(current_gps, x = 0, y = 0, z = 0, theta = 0):
    '''
    Transforms a vector defined in the XYZ coordinates to [lon, lat, alt] given a refference GPS position.
    For theta == 0 => X == East, Y == North, Z == Up
    All angles are in degrees.
    current_gps can be [lon, lat] or [lon, lat, alt]
    returns [lon, lat, alt]
    '''
    
    # For angles increasing clockwise (iff East == 90*, West == -90*)
    # Otherwise comment out.
    theta = -theta

    # Convert lon, lat from degrees to radians        
    for i in range(2):
        current_gps[i] = math.radians(current_gps[i])
    theta = math.radians(theta)

    # Add alt = 0 in GPS position, if it's missing
    if len(current_gps) < 3:
        current_gps.append(0)

    # Rotate coordinates such that x points East, y points North.
    rotXYZtoENU = numpy.array([[math.cos(theta), -math.sin(theta), 0],
                               [math.sin(theta),  math.cos(theta), 0],
                               [0              ,  0              , 1]])
    ENU = numpy.dot(rotXYZtoENU, numpy.array([[x],[y],[z]]) )
    
    # Constants needed for calculations
    a = 6378137      # Equatorial Earth radius
    b = 6356752.3142 # Polar Earth radius
    
    f = (a - b) / a  # flattening
    esq = 2*f-f**2   # eccenctricity squared
    N = a/(math.sqrt(1 - (esq * math.sin(current_gps[1])**2))) # Radius of curvature in prime vertical


    #1. convert from gps to ECEF (Earth-Centered Earth-Fixed)
    ECEF = numpy.array([[(N + current_gps[2]) * math.cos(current_gps[1]) * math.cos(current_gps[0])],
                        [(N + current_gps[2]) * math.cos(current_gps[1]) * math.sin(current_gps[0])],
                        [(N * (1 - esq) + current_gps[2]) * math.sin(current_gps[1])]])

    #2. ENU (East, North, Up) -> ECEF
    rotENUtoECEF = numpy.array([[-math.sin(current_gps[0]), -math.sin(current_gps[1])* math.cos(current_gps[0]), math.cos(current_gps[1])* math.cos(current_gps[0])],
                                [ math.cos(current_gps[0]), -math.sin(current_gps[1])* math.sin(current_gps[0]), math.cos(current_gps[1])* math.sin(current_gps[0])],
                                [0                        ,  math.cos(current_gps[1])                          , math.sin(current_gps[1])]])
    pos = numpy.dot(rotENUtoECEF, ENU) + ECEF

    #3. ECEF -> gps
    p = math.sqrt(pos[0,0]**2 + pos[1,0]**2)
    theta = math.atan((pos[2,0]*a)/(p*b))
    eprimesq = (a**2 - b**2)/b**2
                                    
    lat = math.atan( (pos[2,0] + eprimesq * b * math.sin(theta)**3)/(p - esq * a * math.cos(theta)**3) )
    lon = math.atan2(pos[1,0], pos[0,0])
    alt = ( p / math.cos(current_gps[1]) ) - N

    lat = math.degrees(lat)
    lon = math.degrees(lon)

    return [lon, lat, alt]
