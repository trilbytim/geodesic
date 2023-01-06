import numpy as np
from ..basicgeo import P3, AlongAcc, I1
from . import stlgenerator

class TriangleNode:   # replace with just P3
    def __init__(self, p, i, thick = 0):
        self.p = p
        self.i = i  # index
        self.thick = thick #thickness at that node

class TriangleBar:
    def __init__(self, nodeback, nodefore):
        self.nodeback = nodeback
        self.nodefore = nodefore
        self.barforeright = None
        self.barbackleft = None
        self.faceleft = None
        self.faceright = None
        self.badedge = False
        self.i = -1 # index
        assert nodefore.i > nodeback.i
        
    def SetForeRightBL(self, bforeright, bar):
        if bforeright:
            self.barforeright = bar
        else:
            self.barbackleft = bar
    
    def SetFaceRightBL(self, bforeright, f):
        if bforeright:
            self.faceright = f
        else:
            self.faceleft = f

    def GetForeRightBL(self, bforeright):
        if bforeright:
            return self.barforeright
        else:
            return self.barbackleft
            
    def GetNodeFore(self, bfore):
        if bfore:
            return self.nodefore
        else:
            return self.nodeback

    def DGetCellMarkRightL(self, bright):
        if bright:
            return self.cellmarkright
        else:
            return self.cellmarkleft

    def DIsTriangleRefBar(self):
        return self.barforeright and (self.barforeright.GetNodeFore(self.nodefore == self.barforeright.nodeback).i > self.nodeback.i)
    
class Face:
    def __init__(self, nodes, normal, i):
        assert len(nodes)==3, 'Must supply indexes of 3 adjoining nodes'
        self.nodes = nodes #list of nodes
        self.bars = [] #list of bars
        self.normal = normal #Vector of normal to face
        self.i = i  # index
        

class TriangleBarMesh:
    def __init__(self, fname=None, trans=None, flat9triangles=None):
        self.nodes = [ ]
        self.bars = [ ]
        self.faces = [ ]
        #self.xlo, self.xhi, self.ylo, self.yhi  # set in NewNode()
        self.ntriangles = 0
        self.meshsize = 0
        
        if fname is not None:
            sr = stlgenerator.stlreader(fname, trans)
            self.BuildTriangleBarmesh(sr)
            r0 = max(map(abs, (self.xlo, self.xhi, self.ylo, self.yhi, self.zlo, self.zhi)))
            assert r0 < 100000, ("triangles too far from origin", r0)
        elif flat9triangles:
            self.BuildTriangleBarmesh(flat9triangles)
            r0 = max(map(abs, (self.xlo, self.xhi, self.ylo, self.yhi, self.zlo, self.zhi)))
            assert r0 < 100000, ("triangles too far from origin", r0)
        
    def GetNodePoint(self, i):
        return self.nodes[i].p
    def GetBarPoints(self, i):
        bar = self.bars[i]
        return (bar.nodeback.p, bar.nodefore.p)
    def GetTriPoints(self, i):
        bar = self.bars[i]
        node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
        return (bar.nodeback.p, bar.nodefore.p, node2.p)
    
    def GetVertices(self): #return a numpy array of all vertices suitable for plotting in Polyscope
        V = []
        for node in self.nodes:
            V.append(((node.p.x),(node.p.y),(node.p.z)))
        return np.array(V)
    
    def GetFaces(self): #return a numpy array of all faces suitable for plotting in Polyscope
        F = []
        for face in self.faces:
            F.append((face.nodes[0].i,face.nodes[1].i,face.nodes[2].i))
        return np.array(F)
    
    def GetNormals(self): #return a numpy array of all faces suitable for plotting in Polyscope
        N = []
        for face in self.faces:
            N.append(face.normal)
        return np.array(N)
    
    def GetEdgeBars(self):#return a list of bars that are on an edge
        EB =[]
        for bar in self.bars:
            if bar.barforeright == None or bar.barbackleft == None:
                EB.append(bar)
        return EB
        
    def NewNode(self, p):
        if self.nodes:
            if p.x < self.xlo:  self.xlo = p.x
            if p.x > self.xhi:  self.xhi = p.x
            if p.y < self.ylo:  self.ylo = p.y
            if p.y > self.yhi:  self.yhi = p.y
            if p.z < self.zlo:  self.zlo = p.z
            if p.z > self.zhi:  self.zhi = p.z
        else:
            self.xlo = self.xhi = p.x
            self.ylo = self.yhi = p.y
            self.zlo = self.zhi = p.z
        self.nodes.append(TriangleNode(p, len(self.nodes)))
        return self.nodes[-1]

        
    def BuildTriangleBarmesh(self, trpts):
        # strip out duplicates in the corner points of the triangles
        ipts, jtrs, nvecs = [ ], [ ], [ ]
        
        for i, tr in enumerate(trpts):
            ipts.append((tr[0:3], i*3+0))
            ipts.append((tr[3:6], i*3+1))
            ipts.append((tr[6:9], i*3+2))
            nvecs.append(P3(tr[9],tr[10],tr[11]))
            jtrs.append([-1, -1, -1])
            self.ntriangles += 1
        
        ipts.sort()
        prevpt = None
        for pt, i3 in ipts:
            if not prevpt or prevpt != pt:
                self.NewNode(P3(pt[0], pt[1], pt[2]))
                prevpt = pt
            jtrs[i3 // 3][i3 % 3] = len(self.nodes) - 1
        del ipts
            
        
        # create the barcycles around each triangle
        tbars = [ ]
        for jt0, jt1, jt2 in jtrs:
            if jt0 != jt1 and jt0 != jt2 and jt1 != jt2:  # are all the points distinct?
                self.faces.append(Face(nodes = (self.nodes[jt0],self.nodes[jt1],self.nodes[jt2]),normal = nvecs[len(self.faces)], i = len(self.faces)))
                tbars.append(jt0 < jt1 and TriangleBar(self.nodes[jt0], self.nodes[jt1]) or TriangleBar(self.nodes[jt1], self.nodes[jt0]))
                tbars.append(jt1 < jt2 and TriangleBar(self.nodes[jt1], self.nodes[jt2]) or TriangleBar(self.nodes[jt2], self.nodes[jt1]))
                tbars.append(jt2 < jt0 and TriangleBar(self.nodes[jt2], self.nodes[jt0]) or TriangleBar(self.nodes[jt0], self.nodes[jt2]))
                tbars[-3].SetForeRightBL(jt0 < jt1, tbars[-2]) 
                tbars[-2].SetForeRightBL(jt1 < jt2, tbars[-1]) 
                tbars[-1].SetForeRightBL(jt2 < jt0, tbars[-3])
                tbars[-3].SetFaceRightBL(jt0 < jt1, self.faces[-1].i) 
                tbars[-2].SetFaceRightBL(jt1 < jt2, self.faces[-1].i) 
                tbars[-1].SetFaceRightBL(jt2 < jt0, self.faces[-1].i)

        del jtrs
        
        # strip out duplicates of bars where two triangles meet
        tbars.sort(key=lambda bar: (bar.nodeback.i, bar.nodefore.i, not bar.barbackleft))
        prevbar = None
        for bar in tbars:
            if prevbar and prevbar.nodeback == bar.nodeback and prevbar.nodefore == bar.nodefore and \
                    prevbar.barbackleft and not prevbar.barforeright and not bar.barbackleft and bar.barforeright:
                prevbar.barforeright = bar.barforeright
                node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
                bar2 = bar.barforeright.GetForeRightBL(bar.nodefore == bar.barforeright.nodeback)
                assert bar2.GetNodeFore(bar2.nodeback == node2) == bar.nodeback
                assert bar2.GetForeRightBL(bar2.nodeback == node2) == bar
                bar2.SetForeRightBL(bar2.nodeback == node2, prevbar)
                prevbar = None
                if self.bars[-1].faceleft or self.bars[-1].faceleft ==0:
                    self.bars[-1].faceright = bar.faceright
                else:
                    self.bars[-1].faceleft = bar.faceleft
            else:
                prevbar = bar
                assert prevbar.i == -1
                prevbar.i = len(self.bars)
                self.bars.append(bar)
        del tbars
        
        for bar in self.bars:
            if bar.faceleft or bar.faceleft == 0:
                self.faces[bar.faceleft].bars.append(bar)
            if bar.faceright or bar.faceright == 0:
                self.faces[bar.faceright].bars.append(bar)
                
        
        for b in self.bars:
            self.meshsize += P3.Len(b.GetNodeFore(True).p-b.GetNodeFore(False).p)/len(self.bars)

        if __debug__:
            Dntriangles = 0
            for bar in self.bars:
                if bar.barforeright:
                    node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
                    if node2.i > bar.nodeback.i:
                        Dntriangles += 1
            assert self.ntriangles == Dntriangles
        print('Triangle bar mesh loaded with',self.ntriangles, 'triangles of average size','%s' % float('%.3g' % self.meshsize))

    def GetBarMeshTriangles(self, flat9s=False):
        tris = [ ]
        for bar in self.bars:
            if bar.barforeright:
                node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
                if node2.i > bar.nodeback.i:
                    node0 = bar.nodeback
                    node1 = bar.nodefore
                    if flat9s:
                        tris.append((node0.p.x, node0.p.y, node0.p.z, node1.p.x, node1.p.y, node1.p.z, node2.p.x, node2.p.y, node2.p.z))
                    else:
                        tris.append((node0.p, node1.p, node2.p))
        return tris
            
       
class SingleBoxedTriangles:  # this is its own single box
    def __init__(self, tbarmesh):
        self.tbarmesh = tbarmesh
        self.pointis = list(range(len(tbarmesh.nodes)))
        self.edgeis = list(range(len(tbarmesh.bars)))
        self.triangleis = [ ]
        for iedge, bar in enumerate(tbarmesh.bars):
            if bar.barforeright:
                node2 = bar.barforeright.GetNodeFore(bar.nodefore == bar.barforeright.nodeback)
                if node2.i > bar.nodeback.i:
                    self.triangleis.append(iedge)
                    
    def CloseBoxeGenerator(self, xlo, xhi, ylo, yhi, r):
        return [0]
    def GetTriangleBox(self, ixy):
        assert ixy == 0
        return self

    def GetNodePoint(self, i):
        return self.tbarmesh.GetNodePoint(i)
    def GetBarPoints(self, i):
        return self.tbarmesh.GetBarPoints(i)
    def GetTriPoints(self, i):
        return self.tbarmesh.GetTriPoints(i)
    
    def SlicePointisZ(self, pointis, zlo, zhi):
        return pointis
