from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
import math

def Square(X):
    return X*X
def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X, msg)

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

def TriangleNodeOpposite(bar, bGoRight):
    bartop = bar.GetForeRightBL(bGoRight)
    if bartop != None:
        return bartop.GetNodeFore(bartop.nodeback == bar.GetNodeFore(bGoRight))
    return None
    
def GeoCrossBar(c, bar, lam, bGoRight):
    Na, Nb = bar.nodeback, bar.nodefore
    d = Na.p + (Nb.p - Na.p)*lam
    Ne = TriangleNodeOpposite(bar, bGoRight)
    if Ne == None:
        print("GeoCrossBar fail", Na.p, Nb.p, lam, bGoRight, c)
        print(bar.barforeright, bar.barbackleft)
        return (None, None, 0.0, False)
    bAEcrossing, q, Gx, bEnd = GeoCrossAxis(Na.p, Nb.p, c, lam, Ne.p)
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
    return (d, bar, lam, bGoRight)



