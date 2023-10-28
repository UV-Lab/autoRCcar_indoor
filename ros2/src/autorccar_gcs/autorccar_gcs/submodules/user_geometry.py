from re import X
import numpy as np
import math

def quat2eulr(quat):
    quat = quat / np.linalg.norm(quat)

    # /example/test_ekf 실행시(w,x,y,z 순으로 저장)
    a = quat[0]
    b = quat[1]
    c = quat[2]
    d = quat[3]

    # NavTest 데이터 (ROS quaternion x,y,z,w 순으로 저장)
    # a = quat[3]
    # b = quat[0]
    # c = quat[1]
    # d = quat[2]

    A = 2*(a*b + c*d)
    B = a*a - b*b - c*c + d*d
    C = 2*(b*d - a*c)
    D = 2*(a*d + b*c)
    E = a*a + b*b - c*c - d*d

    phi = math.atan2(A,B)  
    theta = math.asin(-C)
    psi = math.atan2(D,E)

    euler = [phi, theta, psi] # rad

    return euler


def llh2ned(llh, llh_ori):
    aa = 6378317.0
    ee = 0.0818191908426

    lat = llh[0]
    lon = llh[1]
    hei = llh[2]
    lat_ori = llh_ori[0]
    lon_ori = llh_ori[1]
    hei_ori = llh_ori[2]

    Rtmp = 1 - ee*ee*math.sin(lat_ori)*math.sin(lat_ori)

    RM = aa*(1 - ee*ee)/(Rtmp**(3/2))
    RN = (aa) / (Rtmp**(1/2))

    n = (RM+hei)*(lat - lat_ori)
    e = (RN+hei)*math.cos(lat)*(lon - lon_ori)
    d = -(hei - hei_ori)

    return [n, e, d]


def ned2llh(ned, llh_ori):
    aa = 6378317.0000
    ee = 0.0818191908426

    n = ned[0]
    e = ned[1]
    d = ned[2]
    lat_ori = llh_ori[0]
    lon_ori = llh_ori[1]
    hei_ori = llh_ori[2]

    Rtmp = 1 - ee*ee*math.sin(lat_ori)*math.sin(lat_ori)

    RM = aa*(1 - ee*ee)/(Rtmp**(3/2))
    RN = (aa) / (Rtmp**(1/2))

    hei = -d + hei_ori
    lat = n/(RM+hei) + lat_ori
    lon = e/((RN+hei)*math.cos(lat)) + lon_ori

    return [lat, lon, hei]


def xyz2llh(xyz):
    
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]
    x2 = x*x
    y2 = y*y
    z2 = z*z

    aa = 6378317.0000
    bb = 6356752.3142
    ee = 0.0818191908426  # sqrt(1-(bb/aa)**2)  

    b2 = bb*bb
    e2 = ee*ee
    ep = ee*(aa/bb)
    rr = math.sqrt(x2+y2)
    r2 = rr*rr
    E2 = aa*aa - bb*bb
    FF = 54*b2*z2
    GG = r2 + (1-e2)*z2 - e2*E2
    cc = (e2*e2*FF*r2)/(GG*GG*GG)
    ss = (1 + cc + math.sqrt(cc*cc + 2*cc))**(1/3)
    PP = FF / (3 * (ss+1/ss+1)**2 * GG*GG)
    QQ = math.sqrt(1+2*e2*e2*PP)
    ro = -(PP*e2*rr)/(1+QQ) + math.sqrt((aa*aa/2)*(1+1/QQ) - (PP*(1-e2)*z2)/(QQ*(1+QQ)) - PP*r2/2)

    tmp = (rr - e2*ro)**2
    UU = math.sqrt(tmp + z2)
    VV = math.sqrt(tmp + (1-e2)*z2)
    zo = (b2*z)/(aa*VV)

    hei = UU*(1 - b2/(aa*VV))
    lat = math.atan((z + ep*ep*zo)/rr)

    temp = math.atan(y/x)

    if x >= 0:
        lon = temp
    elif (x < 0) & (y >= 0):
        lon = math.pi + temp
    else:
        lon = temp - math.pi
    
    return [lat, lon, hei]
