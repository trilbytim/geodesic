import re

def linapply(y, brel):
    for e in y.split(","):
        k, v = e.split()
        if brel:
            P[k] += float(v)
        else:
            P[k] = float(v)

d = 'output/'
fname = "A_40mm_helical_45_10"
ftext = open(d+fname+'.src', "r").read()
P = { "E3":0.0, "E4":0.0 }

reforblk = "(?s)LIN \{(.*?)\}\nHALT\n.*?FOR N_CYC=1 to (\d+)\n(.*?)ENDFOR"
forblks = re.findall(reforblk, ftext)

Pblocks = [ ]
for bknum, forblk in enumerate(forblks):
    P["E3"] = P["E4"] = 0
    linapply(forblk[0], False)
    Ps = [ P.copy() ]
    lins = re.findall("LIN_REL \{(.*?)\}", forblk[2])
    k1 = Ps[-1]
    Ncycles = int(forblk[1])
    for N_CYC in range(Ncycles):
        for lin in lins:
            linapply(lin, True)
            Ps.append(P.copy())
    Pblocks.append({"Ps":Ps, "Ncycles":Ncycles})
    
res=[]
first = True
for P in Pblocks[0]['Ps']:
    if first:
        res.append("LIN {X %+.3f,Y %+.3f,Z %+.3f,A %+.3f,E1 %+.3f,E3 %+.3f,E4 %+.3f}\n" % \
                   (P['X'], P['Y'], 0.0, P['A'], P['E1'], P['E3'], P['E4']))
        first = False
    else:
        res.append("LIN {X %+.3f,Y %+.3f,Z %+.3f,A %+.3f,E1 %+.3f,E3 %+.3f,E4 %+.3f} C_DIS\n" % \
                       (P['X'], P['Y'], 0.0, P['A'], P['E1'], P['E3'], P['E4']))
        
fout = open(d+fname+'-LIN.src', "w")
fout.write(open("srcheader-HUMM.txt").read())
fout.write(res[0])
fout.write("HALT\n")
fout.write("ACTION(#HUMM3_START,100,1000,#STOPONERROR,ERROR_CODE)\n")
fout.write("$APO.CDIS=85\n")
fout.write("$VEL_EXTAX[1]=100\n")
for l in res[1:]:
    fout.write(l)
fout.write("HALT\n")
fout.write("END\n");
fout.close()