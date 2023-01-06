# Utilities for generating geodesic paths across meshes
import numpy as np
import math
from barmesh.basicgeo import P3
from barmesh.tribarmes import trianglebarmesh

def pointOnEdge(i, l):
    pt1 = tbm.bars[i].GetNodeFore(True).p
    pt2 = tbm.bars[i].GetNodeFore(False).p
    vec12 = (pt2[0]-pt1[0],pt2[1]-pt1[1],pt2[2]-pt1[2])
    ptl = P3(pt1[0]+l*vec12[0],pt1[1]+l*vec12[1],pt1[2]+l*vec12[2])
    return ptl

def P3list2array(P3list):
    pts =[]
    for p in P3list:
        pts.append((p.x,p.y,p.z))
    return np.array(pts)


def find_rot(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """

    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

#function to rotate a P3 vector about an axis (of any length) and an angle in degrees
def rotAxisAngle(v,ax,a):
    ax = P3.ZNorm(ax)
    c = np.cos(np.radians(a))
    s = np.sin(np.radians(a))
    C = 1 - np.cos(np.radians(a))
    Q= np.array(((ax.x*ax.x*C+c,      ax.x*ax.y*C-ax.z*s, ax.x*ax.z*C+ax.y*s),
                 (ax.y*ax.x*C+ax.z*s, ax.y*ax.y*C+c,      ax.y*ax.z*C-ax.x*s),
                 (ax.z*ax.x*C-ax.y*s, ax.z*ax.y*C+ax.x*s, ax.z*ax.z*C+c)))
    v = np.dot(Q,np.array((v.x,v.y,v.z)))
    return P3(v[0],v[1],v[2])

# axis is a b
# incoming at d = Along(lam, a, b)
# incoming from c to d 
# incoming (d - c).(b - a) = |d-c||b-a|cos(theta) = |d-c|I
# I = |b-a|cos(theta) = (d - c).(b - a)/|d-c|

# triangle on right hand side is to e
# outgoing to x where x = Along(q, a, e) or x = Along(q, b, e)
# Solve (x - d).(b - a) = |x-d||b-a|cos(theta) = |x-d|I

# Set d = 0, v = b - a
# x.v = |x|I  where x = a + (e - a)*q = a + f*q
# x.x I^2 = (x.v)^2

# x.x I^2 = (a + f*q)^2 I^2 = q^2 f^2 I^2 + 2q a.f I^2 + a^2 I^2
#   =
# (x.v)^2 = (a.v + f.v*q)^2 = q^2 (f.v)^2 + 2q (a.v)(f.v) + (a.v)^2
# q^2 ((f.v)^2 - f^2 I^2) + 2q ((a.v)(f.v) - a.f I^2) + (a.v)^2 - a^2 I^2

def Square(X):
    return X*X
def TOL_ZERO(X):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X)

def GeoCrossAxisE(a, Vae, Vab, Isq, Isgn):
    # Solve: Isq*x.Lensq() - Square(P3.Dot(x, Vab)) = 0   for x = a + Vae*q
    # 0 = Isq*(a^2 + 2q a.Vae + q^2 Vae^2) - (a.Vab + Vae.Vab q)^2
    #   = Isq*(a^2 + 2q adf + q^2 Vae^2) - (adv + fdv q)^2

    fdv = P3.Dot(Vae, Vab)
    adv = P3.Dot(a, Vab)
    adf = P3.Dot(a, Vae)
    qA = Square(fdv) - Vae.Lensq()*Isq
    qB2 = adv*fdv - adf*Isq
    qC = Square(adv) - a.Lensq()*Isq
    if abs(qA) <= abs(qB2)*1e-7:
        if qB2 == 0:
            return -1.0
        q = -qC/(2*qB2)
    else:
        qdq = Square(qB2) - qA*qC
        if qdq < 0.0:
            #print("qdq", qdq)
            return -1.0
        qs = math.sqrt(qdq) / qA
        qm = -qB2 / qA
        q = qm + qs*Isgn
    # q = qs +- qm,  x = a + Vae*q,  Dot(x, Vab) same sign as Dot(Vcd, Vab)
    if abs(q) < 100:
        TOL_ZERO(qA*Square(q) + qB2*2*q + qC)
    return q

#
# This is the basic function that crosses from one triangle to the next
#
def GeoCrossAxis(Ga, Gb, Gc, lam, Ge):
    Vab = Gb - Ga
    Gd = Ga + Vab*lam
    Vcd = Gd - Gc
    if Vcd.Len() == 0:
        bEnd = True
        bAEcrossing = False
        q = 0
        Gx = Gc
    else:
        bEnd = False
        cdDab = P3.Dot(Vcd, Vab)
        Isq = Square(cdDab) / Vcd.Lensq()
        Isgn = -1 if cdDab < 0 else 1
        qVae = GeoCrossAxisE(Ga - Gd, Ge - Ga, Vab, Isq, Isgn)
        qVbe = GeoCrossAxisE(Gb - Gd, Ge - Gb, -Vab, Isq, -Isgn)
        bAEcrossing = (abs(qVae - 0.5) < abs(qVbe - 0.5))
        q = qVae if bAEcrossing else qVbe
        Gx = (Ga + (Ge - Ga)*q) if bAEcrossing else (Gb + (Ge - Gb)*q)
        Dx = Gx - Gd
        TOL_ZERO(Isq - Square(P3.Dot(Dx, Vab)/Dx.Len()))
        TOL_ZERO(P3.Dot(Vcd, Vab)/Vcd.Len() - P3.Dot(Dx, Vab)/Dx.Len())
    return bAEcrossing, q, Gx, bEnd

# This is the iterative function that goes from bar to bar through the mesh
# c=from point, bar the crossing edge, lam the point on the bar, and bGoRight is the crossing direction
#
def GeoCrossBar(c, bar, lam, bGoRight, bPrintvals=False):
    bEnd = False
    Na, Nb = bar.nodeback, bar.nodefore
    if bGoRight:
        if bar.barforeright == None:
            bEnd = True
            Ne = trianglebarmesh.TriangleNode(P3(bar.nodefore.p.x+0.01,bar.nodefore.p.y+0.01,bar.nodefore.p.z+0.01),-1)
        else:
            Ne = bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
    else:
        if bar.barbackleft == None:
            bEnd = True
            Ne = trianglebarmesh.TriangleNode(P3(bar.nodeback.p.x+0.01,bar.nodeback.p.y+0.01,bar.nodeback.p.z+0.01),-1)
        else:
            Ne = bar.barbackleft.GetNodeFore(bar.barbackleft.nodeback == bar.nodeback)
    d = Na.p + (Nb.p - Na.p)*lam
    if bPrintvals:
        print("\n", (Na.p, Nb.p, c, lam, Ne.p))
    if not bEnd:
        bAEcrossing, q, Gx, bEnd = GeoCrossAxis(Na.p, Nb.p, c, lam, Ne.p)
    if not bEnd:
        if bGoRight:
            if bAEcrossing:
                bar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == Nb)
                lam = q if bar.nodeback == Na else 1-q
                bGoRight = (bar.nodeback == Na)   
            else:
                bar = bar.barforeright
                lam = q if bar.nodeback == Nb else 1-q
                bGoRight = not (bar.nodeback == Nb)
        else:
            if bAEcrossing:
                bar = bar.barbackleft
                lam = q if bar.nodeback == Na else 1-q
                bGoRight = not (bar.nodeback == Na)
            else:
                bar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == Na)
                lam = q if bar.nodeback == Nb else 1-q
                bGoRight = (bar.nodeback == Nb)
                
        c = bar.nodeback.p + (bar.nodefore.p - bar.nodeback.p)*lam
        TOL_ZERO((c - Gx).Len())
    else:
        bar = None
        c = None
    if bPrintvals:
        print("d,c", (d, c))
    
    return (d, bar, lam, bGoRight)


def clamp(num, min_value, max_value):
        if num < min_value or num > max_value: clamped = True
        else: clamped = False
        num = max(min(num, max_value), min_value)
        return num, clamped

#Function to calculate the shortest distance between point P and line AB
def distPointLine(A,B,P):
    if A != B:
        v = B-A
        x = P3.Dot((P-A), v) / P3.Dot(v, v) #Ratio of distance along line AB of closest point to P
    else:
        v = A
        x = -1
    x, clamped = clamp(x, 0, 1) #Fix point of closest approach to be between the two points
    #print('Point of closest approach:',(A + v*x))
    dist = (A + v*x - P).Len()
    return dist,clamped

#Draw a single geodesic line from a specified point at a specified angle and ref0 A reference direction approximately along the axis of the part    
def createGeoLine(tbm, startPt, startFace, theta, ref0, calc_thick = False, tw = 6.35, maxPathLength = 1000, lim = (np.Inf,np.Inf, np.Inf), tol=1e-4):
    valid = True
    fail = 0
    length = 0
    ds = []
    startVec = rotAxisAngle(ref0,tbm.faces[startFace].normal,theta)
    startVN = P3.Cross(startVec,tbm.faces[startFace].normal)
    
    #find which bar intersects with the start vector
    for b in tbm.faces[startFace].bars:
        p1 = b.GetNodeFore(False).p
        p2 = b.GetNodeFore(True).p
        v = p2-p1
        if (P3.Dot((p1-startPt),startVN) > 0) is not (P3.Dot((p2-startPt),startVN) > 0):
            lam = (P3.Dot(startPt,startVN)-P3.Dot(p1,startVN))/(P3.Dot(p2,startVN)-P3.Dot(p1,startVN))
            nextpt = p1 + v*lam
            if P3.Dot((nextpt-startPt),startVec) > 0:
                bar = b
                bGoRight = bar.faceleft == startFace
                break
    
    pts = [startPt]
    faces = [startFace]
    curvs = [0]
    finished = False
    bar_last = tbm.bars[0]
    nodes = [] # list of nodes that fall within half a tow's width of the geoline
    A = startPt
    if type(calc_thick) == list:
        nodes = [False] * len(thickPts)
    while not finished:
        if bar != None:
            if bGoRight:
                faces.append(bar.faceright)
            else:
                faces.append(bar.faceleft)
            bar_last = bar
            pt, bar, lam, bGoRight = GeoCrossBar(pts[-1], bar, lam, bGoRight)
            pts.append(pt)
            
            #Thickness calc
            if calc_thick and type(calc_thick) != list:
                B = pt
                check_bars = [bar_last]
                i=0
                while i < len(check_bars):
                    nei = []
                    df,c = distPointLine(A,B,check_bars[i].GetNodeFore(True).p)
                    if df < tw/2 and check_bars[i].GetNodeFore(True) not in nodes:
                        nodes.append(check_bars[i].GetNodeFore(True))
                    db,c = distPointLine(A,B,check_bars[i].GetNodeFore(False).p)
                    if db < tw/2 and check_bars[i].GetNodeFore(False) not in nodes:
                        nodes.append(check_bars[i].GetNodeFore(False))

                    if check_bars[i].GetForeRightBL(True) and df < tw/2:
                        nei.append(check_bars[i].GetForeRightBL(True))
                    if check_bars[i].GetForeRightBL(False) and db < tw/2:
                        nei.append(check_bars[i].GetForeRightBL(False))
                    for b in nei:
                        if b not in check_bars:
                            check_bars.append(b)
                    i += 1
                A = B
        
        d = P3.Len(pt-pts[-2])
        length += d
        ds.append(d)
        if bar==None:
            finished =True
            print('edge reached on path', theta, 'deg')
            if bar_last.badedge:
                print('PATH FAIL: bad edge reached')
                valid = False
                fail = 1
        elif length>maxPathLength:
            finished =True
            print('PATH FAIL: max path length reached on path', theta)
            valid = False
            fail = 2
        elif pt.x > lim[0]:
            finished = True
        
        elif pt.y > lim[1] and len(pts)>1:
            l = (lim[1]-pts[-2].y) / (pt.y-pts[-2].y)
            pts[-1] = pts[-2] + (pt-pts[-2])*l
            faces[-1] = faces[-2]
            finished = True
        elif pt.z > lim[2]:
            finished = True
        else:
            finished =False
            
    curvs = [0]
    for i in range(1,len(pts)-1):
        vlast = pts[i] - pts[i-1]
        vnext = pts[i+1] - pts[i]
        curvature = P3.Dot(P3.ZNorm(tbm.faces[faces[i]].normal),P3.ZNorm(vlast))    
        curvs.append(curvature)
    curvs.append(0)

    smcurvs = []
    av = 100 #Number of points forwards and backwards to use in curvature calculation
    for i in range(len(pts)):
        lo = max(0,i-av)
        hi = min(i+av+1,len(pts))
        sm = 0
        for j in range(lo,hi):
            d = 1+(sum(ds[j:i])+sum(ds[i:j]))/tbm.meshsize
            c = curvs[j]
            sm += (c/d**2)/tbm.meshsize
        smcurvs.append(sm)
    if min(smcurvs) < 0:
        print('PATH FAIL: concave area of',min(smcurvs),'on path', theta)
        valid = False
        fail = 3
        
    return {'pts':pts,'curvs':smcurvs,'faces':faces, 'nodes':nodes, 'valid':valid, 'fail':fail,'length':length}


def createDoubleGeoLine(tbm, startPt, startFace, theta, ref0, calc_thick = True, tw = 6.35, maxPathLength = 1000):
    geoline = createGeoLine(tbm, startPt, startFace, theta, ref0, calc_thick = calc_thick, tw = tw, maxPathLength = maxPathLength)
    geolineB = createGeoLine(tbm, startPt, startFace, theta+180, ref0, calc_thick = calc_thick, tw = tw, maxPathLength = maxPathLength)
    for i in range(1,len(geolineB['pts'])):
        geoline['pts'].insert(0,geolineB['pts'][i])
        geoline['faces'].insert(0,geolineB['faces'][i])
        geoline['curvs'].insert(0,geolineB['curvs'][i])
        if geoline['nodes']:
            geoline['nodes'].insert(0,geolineB['nodes'][i])
        geoline['valid'] = geoline['valid'] and geolineB['valid']
        geoline['fail'] = [geoline['fail'],geolineB['fail']]
    return geoline


def createPly(tbm, seeds,thet = None, CPT = 0.125, calc_thick = True, tw = 6.35, maxPathLength = 1000):
    geolines=[]
    thicks = np.zeros(len(tbm.nodes))
    thetas =[]
    for i in range(len(seeds['pts'])):
        if thet:
            theta =thet
        else:
            theta = seeds['thetas'][i]
        thetas.append(theta)
        startPt = seeds['pts'][i]
        ref0 = seeds['ref0s'][i]
        startFace =seeds['faces'][i]
        geoline = createGeoLine(tbm, startPt, startFace, theta, ref0, calc_thick = calc_thick, tw = tw, maxPathLength = maxPathLength)
        if geoline['valid']:
            geolines.append(geoline)
        else:
            print('!!!!INVALID GEOLINE NOT ADDED!!!')
        for n in geoline['nodes']:
            thicks[n.i] += CPT
    return (geolines, thicks, thetas)

            
def findGoodEdge(edgeBars,badEdgeBars):
    goodEdgeBars =[]
    for eb in edgeBars:
        good = True
        for beb in badEdgeBars:
            if eb == beb: good = False
        if good: 
            goodEdgeBars.append(eb)
    return goodEdgeBars

#Function to sort bars into a single continuous line starting from the first one in the unsorted list
def sortBars(tbm, unsortedBars):
    sortedBars = []
    eb1 = unsortedBars[0]
    
    for i in range(len(unsortedBars)):
        for eb2 in unsortedBars:
            if (eb1.GetNodeFore(True) == eb2.GetNodeFore(True) or eb1.GetNodeFore(True) == eb2.GetNodeFore(False) or
                eb1.GetNodeFore(False) == eb2.GetNodeFore(True) or eb1.GetNodeFore(False) == eb2.GetNodeFore(False)):
                #print('match found at',eb2.i)
                sortedBars.append(eb2)
                unsortedBars.remove(eb2)
                eb1 = eb2
                break

    return sortedBars