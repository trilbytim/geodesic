# Project the line by the tangent vector by a fixed

from barmesh.basicgeo import P2, P3

def outputsrclines(pts, freetapelength, robotstate):
    prevE1 = robotstate["E1"]  # used to track the number of complete winds on the eyelet part
    prevE3 = robotstate["E3"]  # used to track the number of complete winds on the lathe part
    res = [ ]

    # create the parallel list of unit vectors for the tape
    # this is separated out because when the angle change is too 
    # great the steps will need to be subdivided somehow as it 
    # swings over the sharp edge following the geodesic 
    # (in line with the double cone whose axis is the edge)
    vecs = [ ]
    for i in range(len(pts)-1):
        vecs.append(P3.ZNorm(pts[i+1] - pts[i]))
        
    # loop through the points and vectors 
    # (we use the incoming vector at each triangle edge, not the outgoing vector)
    for pt, vec in zip(pts[1:], vecs):
        tapevector = vec*freetapelength
        tcp = pt + tapevector
        radialvec = P2(tcp.x, tcp.z)
        lE3 = radialvec.Arg()
        E3 = lE3 + 360*int((abs(prevE3 - lE3)+180)/360)*(1 if prevE3 > lE3 else -1)
        X = -radialvec.Len()
        tangentunitvec = -P2.CPerp(radialvec)*(1/X)
        A = 0.0
        E1vec = P2(-P2.Dot(tangentunitvec, P2(tapevector.x, tapevector.z)), -tapevector.y)
        lE1 = E1vec.Arg()
        E1 = lE1 + 360*int((abs(prevE1 - lE1)+180)/360)*(1 if prevE1 > lE1 else -1)
        
        #if (i%25) == 0:
        #    print(E1, E3, tapevector)
        res.append("LIN {X %+.3f,Y %+.3f,Z %+.3f,A %+.3f,E1 %+.3f,E3 %+.3f,E4 %+.3f} C_DIS\n" % \
                   (X, tcp.y, 0.0, A, E1, E3, 0.0))
        prevE1 = E1
        prevE3 = E3

    robotstate["E1"] = prevE1
    robotstate["E3"] = prevE3
    return res
