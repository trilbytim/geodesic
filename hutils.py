import re

class HPos:
    def __init__(self, P):
        self.X = P["X"]
        self.Y = P["Y"]
        assert P["Z"] == 0.0, P
        self.A = P["A"]
        assert P["B"] == 0.0, P
        assert P["C"] == 0.0, P
        self.E = P["E1"]
        self.H = P["E3"]
        assert P["E3"] == P["E4"], P
        
def linapply(P, y, brel):
    for e in y.split(","):
        k, v = e.split()
        if brel: # and k in "AXYZ":
            P[k] += float(v)
        else:
            P[k] = float(v)


reforblk = "(?s)LIN \{(.*?)\}\nHALT\n.*?PLY[^\(]*\(([^\)]*)\).*?FOR N_CYC=1 to (\d+)\n(.*?)ENDFOR"
def Hparse(fname):
    ftext = open(fname, "r").read()
    forblks = re.findall(reforblk, ftext)

    Pblocks = [ ]
    P = { "E3":0.0, "E4":0.0, "Z":0.0 }
    for bknum, forblk in enumerate(forblks):
        P["E3"] = P["E4"] = 0
        linapply(P, forblk[0], False)
        Ps = [ HPos(P) ]
        #print(P["Y"], forblk[0])
        lins = re.findall("LIN_REL \{(.*?)\}", forblk[3])
        k1 = Ps[-1]
        Ncycles = int(forblk[2])
        for N_CYC in range(Ncycles):
            for lin in lins:
                linapply(P, lin, True)
                Ps.append(HPos(P))
                #assert bknum <= 5 or P.get("E3", 0) < 10000, (bknum, P)
        Pblocks.append({"H":Ps, "plytype":forblk[1], "Ncycles":Ncycles})
    return Pblocks

