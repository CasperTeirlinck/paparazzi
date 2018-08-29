# -*- coding: utf-8 -*-
"""
Created on Sat Apr  7 16:59:54 2018

@author: Leonor Inverno
"""

import numpy as np
from math import *

    
#Conversion variables
nm = 1852 #NM to m
ft = 0.3048 #ft to m
#------------------------------------------------------------------
#Functions from bluesky/tools/geo.py
#------------------------------------------------------------------



#WGS'84 reference frame
def rwgs84(latd):
    """ Calculate the earths radius with WGS'84 geoid definition
        In:  lat [deg] (latitude)
        Out: R   [m]   (earth radius) """
    lat    = np.radians(latd)
    a      = 6378137.0       # [m] Major semi-axis WGS-84
    b      = 6356752.314245  # [m] Minor semi-axis WGS-84
    coslat = np.cos(lat)
    sinlat = np.sin(lat)

    an     = a * a * coslat
    bn     = b * b * sinlat
    ad     = a * coslat
    bd     = b * sinlat

    # Calculate radius in meters
    r = np.sqrt((an * an + bn * bn) / (ad * ad + bd * bd))

    return r

#WGS'84 reference frame in matrix
def rwgs84_matrix(latd):
    """ Calculate the earths radius with WGS'84 geoid definition
        In:  lat [deg] (Vector of latitudes)
        Out: R   [m]   (Vector of radii) """

    lat    = np.radians(latd)
    a      = 6378137.0       # [m] Major semi-axis WGS-84
    b      = 6356752.314245  # [m] Minor semi-axis WGS-84
    coslat = np.cos(lat)
    sinlat = np.sin(lat)
    an     = a * a * coslat
    bn     = b * b * sinlat
    ad     = a * coslat
    bd     = b * sinlat

    anan   = np.multiply(an, an)
    bnbn   = np.multiply(bn, bn)
    adad   = np.multiply(ad, ad)
    bdbd   = np.multiply(bd, bd)
    # Calculate radius in meters
    r      = np.sqrt(np.divide(anan + bnbn, adad + bdbd))

    return r

#Returns the bearing and distance between 2 aircraft   
def qdrdist(latd1, lond1, latd2, lond2):
    """ Calculate bearing and distance, using WGS'84
        In:
            latd1,lond1 en latd2, lond2 [deg] :positions 1 & 2
        Out:
            qdr [deg] = heading from 1 to 2
            d [nm]    = distance from 1 to 2 in nm """

    # Haversine with average radius

    # Check for hemisphere crossing,
    # when simple average would not work

    # res1 for same hemisphere
    res1 = rwgs84(0.5 * (latd1 + latd2))

    # res2 :different hemisphere
    a    = 6378137.0       # [m] Major semi-axis WGS-84
    r1   = rwgs84(latd1)
    r2   = rwgs84(latd2)
    res2 = 0.5 * (abs(latd1) * (r1 + a) + abs(latd2) * (r2 + a)) / \
        (abs(latd1) + abs(latd2))

    # Condition
    sw   = (latd1 * latd2 >= 0.)

    r    = sw * res1 + (1 - sw) * res2

    # Convert to radians
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    lat2 = np.radians(latd2)
    lon2 = np.radians(lond2)

    sin1 = np.sin(0.5 * (lat2 - lat1))
    sin2 = np.sin(0.5 * (lon2 - lon1))

    coslat1 = np.cos(lat1)
    coslat2 = np.cos(lat2)

    root = sin1 * sin1 + coslat1 * coslat2 * sin2 * sin2
    d    =  2.0 * r * np.arctan2(np.sqrt(root) , np.sqrt(1.0 - root))

    #    d =2.*r*np.arcsin(np.sqrt(sin1*sin1 + coslat1*coslat2*sin2*sin2))

    # Bearing from Ref. http://www.movable-type.co.uk/scripts/latlong.html

    qdr = np.degrees(np.arctan2(np.sin(lon2 - lon1) * coslat2,
        coslat1 * np.sin(lat2) - np.sin(lat1) * coslat2 * np.cos(lon2 - lon1)))

    return qdr, d/nm

#Returns the bearing and distance between aircraft 
#in a matrix (multiple aircraft)  
def qdrdist_matrix(lat1, lon1, lat2, lon2):
    """ Calculate bearing and distance vectors, using WGS'84
        In:
            latd1,lond1 en latd2, lond2 [deg] :positions 1 & 2 (vectors)
        Out:
            qdr [deg] = heading from 1 to 2 (matrix)
            d [nm]    = distance from 1 to 2 in nm (matrix) """
    prodla =  lat1.T * lat2
    condition = prodla < 0

    r = np.zeros(prodla.shape)
    r = np.where(condition, r, rwgs84_matrix(lat1.T + lat2))

    a = 6378137.0

    r = np.where(np.invert(condition), r, (np.divide(np.multiply
      (0.5, ((np.multiply(abs(lat1), (rwgs84_matrix(lat1)+a))).T +
         np.multiply(abs(lat2), (rwgs84_matrix(lat2)+a)))),
            (abs(lat1)).T+(abs(lat2)+(lat1 == 0.)*0.000001))))  # different hemisphere

    diff_lat = lat2-lat1.T
    diff_lon = lon2-lon1.T

    sin1 = (np.radians(diff_lat))
    sin2 = (np.radians(diff_lon))

    sinlat1 = np.sin(np.radians(lat1))
    sinlat2 = np.sin(np.radians(lat2))
    coslat1 = np.cos(np.radians(lat1))
    coslat2 = np.cos(np.radians(lat2))

    sin10 = np.mat(np.abs(np.sin(sin1/2.)))
    sin20 = np.mat(np.abs(np.sin(sin2/2.)))
    sin1sin1 =  np.multiply(sin10, sin10)
    sin2sin2 =  np.multiply(sin20, sin20)
    sqrt =  sin1sin1+np.multiply((coslat1.T*coslat2), sin2sin2)

    dist_c =  np.multiply(2., np.arctan2(np.sqrt(sqrt), np.sqrt(1-sqrt)))
    dist = np.multiply(r/nm, dist_c)
    #    dist = np.multiply(2.*r, np.arcsin(sqrt))

    sin21 = np.mat(np.sin(sin2))
    cos21 = np.mat(np.cos(sin2))
    y = np.multiply(sin21, coslat2)

    x1 = np.multiply(coslat1.T, sinlat2)

    x2 = np.multiply(sinlat1.T, coslat2)
    x3 = np.multiply(x2, cos21)
    x = x1-x3

    qdr = np.degrees(np.arctan2(y, x))

    return qdr, dist

def qdrpos(latd1, lond1, qdr, dist):
    """ Calculate vector with positions from vectors of reference position,
        bearing and distance.
        In:
             latd1,lond1  [deg]   ref position(s)
             qdr          [deg]   bearing (vector) from 1 to 2
             dist         [nm]    distance (vector) between 1 and 2
        Out:
             latd2,lond2 (IN DEGREES!)
        Ref for qdrpos: http://www.movable-type.co.uk/scripts/latlong.html """

    # Unit conversion
    R = rwgs84(latd1)/nm
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)

    # Calculate new position
    lat2 = np.arcsin(np.sin(lat1)*np.cos(dist/R) +
              np.cos(lat1)*np.sin(dist/R)*np.cos(np.radians(qdr)))

    lon2 = lon1 + np.arctan2(np.sin(np.radians(qdr))*np.sin(dist/R)*np.cos(lat1),
                     np.cos(dist/R) - np.sin(lat1)*np.sin(lat2))
    return np.degrees(lat2), np.degrees(lon2)
