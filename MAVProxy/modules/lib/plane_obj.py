
# planeobjects - a collection of graphical plane objects using visual
# Tom Fetter @2015
# version 1.1

from visual import *
from math import *

newframe=frame  # needed to avoid "TypeError: 'frame' object is not callable" 

class bodytube(object):
    def __init__(self, frame=None, Dr=.1, Lbt=.66, L1=.33, pos=(0,0,0), axis=(0,0,1), color=color.red):
        self._Dr = Dr
        self._Lbt = Lbt
        self._L1 = L1
        self._pos = pos
        self._axis = axis
        self._color = color

        L2 = Lbt - self._L1
        
        self._btframe = newframe(frame=frame, pos=pos, axis=axis)
        self._body1 = cylinder(frame=self._btframe, pos=(0,0,0),axis=(L1,0,0),radius=Dr/2,color=color)
        self._body2 = cylinder(frame=self._btframe, pos=(0,0,0),axis=(-L2,0,0),radius=Dr/2,color=color)
        
    @property
    def Dr(self):
        return self._Dr
    @Dr.setter
    def Dr(self,Dr):
        self._Dr=Dr
        self._body1.radius = Dr/2 
        self._body2.radius = Dr/2 
       
    @property
    def Lbt(self):
        return self._Lbt
    @Lbt.setter
    def Lbt(self,Lbt):
        self._Lbt=Lbt
        L2 = Lbt-self._L1
        self._body2.axis = (-L2,0,0)
        
    @property
    def L1(self):
        return self._L1
    @L1.setter
    def L1(self,L1):
        self._L1=L1
        L2 = self._Lbt-self._L1
        self._body1.axis = (L1,0,0)
        self._body2.axis = (-L2,0,0)
                
    @property
    # pos moves the bodytube centered abouot the CG location
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self,pos):
        self._pos=pos
        self._btframe.pos = pos
        
    @property
    # axis is aligned along the length of the bodytube
    def axis(self):
        return self._axis
    @axis.setter
    def axis(self,axis):
        self._axis=axis
        self._btframe.axis = axis

    @property
    def color(self):
        return self._color
    @color.setter
    def color(self,color):
        self._color=color
        self._body1.color=color
        self._body2.color=color
 

class fin(object):
    def __init__(self, frame=None, cr=.12, ct=.08, s=.12, Tf=.01, pos=(0,0,0), axis=(0,0,1),
                 up=(1,0,0), color=color.red):

        self._cr = cr
        self._ct = ct
        self._s = s
        self._Tf = Tf
        self._pos = pos
        self._axis = axis
        self._up = up
        self._color = color
     
        self._finframe = newframe(frame=frame, pos=pos, axis=axis, up=up)
        path = [(0,0,-Tf/2),(0,0,Tf/2)]
        self._fin = extrusion(frame=self._finframe, pos=path, shape=self.trapezoid(cr,ct,s), color=self.color)

    def trapezoid(self,cr,ct,s):
        # Shapes are Polygon objects and not VPython objects.  Shapes appears not to have a method to pass 
        # parameters directly through to trapezoid, so self.trapezoid.width = cr does not work, and a call to
        # shaps.trapezoid is required to change th eshape.  This functionjust simplifies the call to those
        # parametes that need to be changed.
        trapezoid = shapes.trapezoid(pos=(-cr/2,s/2), width=cr, height=s, top=ct, rotate=0)
        return trapezoid
        
    @property
    def cr(self):
        return self._cr
    @cr.setter
    def cr(self,cr):
        self._cr=cr
        self._fin.shape = self.trapezoid(cr,self._ct,self._s)

    @property
    def ct(self):
        return self._ct
    @ct.setter
    def ct(self,ct):
        self._ct=ct
        self._fin.shape = self.trapezoid(self._cr,ct,self._s)
        
    @property
    def s(self):
        return self._s
    @s.setter
    def s(self,s):
        self._s=s
        self._fin.shape = self.trapezoid(self._cr,self._ct,s)
        
    @property
    def Tf(self):
        return self._Tf
    @Tf.setter
    def Tf(self,Tf):
        self._Tf=Tf
        path = [(0,0,-Tf/2),(0,0,Tf/2)]
        self._fin.pos = path

    @property
    # pos moves the fin referenced to the trailing root edge
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self,pos):
        self._pos=pos
        self._finframe.pos = pos
        
    @property
    # Axis is aligned with the root edge of the fin (along th elength of the plane)
    def axis(self):
        return self._axis
    @axis.setter
    def axis(self,axis):
        self._axis=axis
        self._finframe.axis = axis

    @property
    # Up is aligned with the radial span of the fin
    def up(self):
        return self._up
    @up.setter
    def up(self,up):
        self._up=up
        self._finframe.up = up

    @property
    def color(self):
        return self._color
    @color.setter
    def color(self,color):
        self._color=color
        self._fin.color=color
        

class nosecone(object):
    def __init__(self, frame=None, Dr=.1, Lnc=.33, pos=(0,0,0), axis=(1,0,0), color=color.red):
        self._Dr = Dr
        self._Lnc = Lnc
        self._pos = pos
        self._axis = axis
        self._color = color
        
        self._noseconeframe = newframe(frame=frame, pos=pos)
        path = paths.circle(pos=(0,0,0), radius=.00001, up=(0,0,1))
        self._nosecone = extrusion(frame=self._noseconeframe, pos=path, shape=self.ogive(Dr,Lnc), color=color)

    def ogive(self,Dr,Lnc):
        # This defines the shape of the tangent ogive nosecone
        n=100
        rnc=Dr/2
        s=[0 for x in range(0,n+1)]
        for i in range(0,n+1):
            z = self.Lnc*float(i)/float(n)
            rnc = (sqrt(-2*(Dr/2)**2*Lnc**2 + 8*(Dr/2)**2*Lnc*z - 4*(Dr/2)**2*z**2 + Lnc**4 + (Dr/2)**4) + (Dr/2)**2 - Lnc**2) / Dr
            s[i] = (rnc,self._Lnc-z)
        return s

    @property
    def Dr(self):
        return self._Dr
    @Dr.setter
    def Dr(self,Dr):
        self._Dr=Dr
        self._nosecone.shape = self.ogive(Dr,self._Lnc)
        
    @property
    def Lnc(self):
        return self._Lnc
    @Lnc.setter
    def Lnc(self,Lnc):
        self._Lnc=Lnc
        self._nosecone.shape = self.ogive(self._Dr,Lnc) 

    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self,pos):
        self._pos=pos
        self._noseconeframe.pos = pos
        
    @property
    # Axis is aligned with the axis of the nosecone
    def axis(self):
        return self._axis
    @axis.setter
    def axis(self,axis):
        self._axis=axis
        self._noseconeframe.axis = axis

    @property
    def color(self):
        return self._color
    @color.setter
    def color(self,color):
        self._color=color
        self._nosecone.color=color

class plane(object):
    def __init__(self, frame=None, Dr=.1,Lbt=.66, Lnc=.33, cr=.6, ct= .3, s=.7, Tf=.01, LCG=.7,
                 pos=(0,0,0), axis=(1,0,0), up=(0,0,-1), CGtrail=True, Tiptrail=False):
        
        self._Dr = Dr                
        self._Lbt = Lbt
        self._Lnc = Lnc
        self._LCG = LCG
        self._cr = cr
        self._ct = ct
        self._s = s
        self._Tf = Tf
        self._pos = pos
        self._axis = axis
        self._up = up

        self._L1 = LCG - Lnc
        self._L2 = Lbt - self._L1

        #two frames are used to align the plane's z azis with "axis" and the x axis with "up"             
        self._frame1 = newframe(frame=frame, axis=axis, up=up)  
        self._frame2 = newframe(frame=self._frame1, axis=(0,1,0), up=(0,0,1))
        
        # Build the plane
        self._bodytube=bodytube(frame=self._frame2, Dr=Dr, Lbt=Lbt, L1=self._L1, pos=(0,0,0),axis=(0,0,1),color=color.red)
        self._nosecone = nosecone(frame=self._frame2, Dr=Dr, Lnc=Lnc, pos=vector(0,0,self._L1), color=color.yellow)
        self._fin1 = fin(frame=self._frame2, cr=cr, ct=ct, s=s, Tf=Tf, pos=vector(0,Dr/2,-self._L2), axis=(0,0,1), up=(0,1,0), color=color.red)
        self._fin2 = fin(frame=self._frame2, cr=cr/4, ct=ct/4, s=s/2, Tf=Tf, pos=vector(-Dr/2,0,-self._L2), axis=(0,0,1), up=(1,0,0), color=color.yellow)
        self._fin3 = fin(frame=self._frame2, cr=cr, ct=ct, s=s, Tf=Tf, pos=vector(0,-Dr/2,-self._L2), axis=(0,0,1), up=(0,-1,0), color=color.red)

        #define the markers
        self._CGtrail = curve(color = color.yellow)
        self._CGtrail.CGtrail = CGtrail       # switch (True/False) to turn marker on/off
        self._Tiptrail = curve(color = color.white)
        self._Tiptrail.Tiptrail = Tiptrail    # switch (True/False) to turn marker on/off
  
    @property
    def Dr(self):
        return self._Dr
    @Dr.setter
    def Dr(self,Dr):
        self._Dr=Dr
        self._bodytube.Dr = Dr
        self._nosecone.Dr = Dr
        self._fin1.pos = vector(Dr/2,0,-self._L2)    
        self._fin2.pos = vector(0,Dr/2,-self._L2)      
        self._fin3.pos = vector(-Dr/2,0,-self._L2)

    @property
    def Lbt(self):
        return self._Lbt
    @Lbt.setter
    def Lbt(self,Lbt):
        self._Lbt=Lbt
        self._L1 = self._LCG-self._Lnc
        self._L2 = Lbt-self._L1
        self._bodytube.Lbt = Lbt
        self._nosecone.pos = vector(0,0,self._L1)
        self._fin1.pos = vector(self._Dr/2,0,-self._L2)    
        self._fin2.pos = vector(0,self._Dr/2,-self._L2)      
        self._fin3.pos = vector(-self._Dr/2,0,-self._L2)  

    @property
    def Lnc(self):
        return self._Lnc
    @Lnc.setter
    def Lnc(self,Lnc):
        self._Lnc=Lnc
        self._L1 = self._LCG-Lnc
        self._L2 = self._Lbt-self._L1
        self._bodytube.L1 = self._L1
        self._nosecone.pos = vector(0,0,self._L1) 
        self._nosecone.Lnc = Lnc
        self._fin1.pos = vector(self._Dr/2,0,-self._L2/4)    
        self._fin2.pos = vector(0,self._Dr/2,-self._L2)      
        self._fin3.pos = vector(-self._Dr/2,0,-self._L2/4)  

    @property
    def cr(self):
        return self._cr
    @cr.setter
    def cr(self,cr):
        self._cr=cr
        self._fin1.cr = cr
        self._fin2.cr = cr/4
        self._fin3.cr = cr

    @property
    def ct(self):
        return self._ct
    @ct.setter
    def ct(self,ct):
        self._ct=ct
        self._fin1.ct = ct
        self._fin2.ct = ct/4
        self._fin3.ct = ct
    
    @property
    def s(self):
        return self._s
    @s.setter
    def s(self,s):
        self._s=s
        self._fin1.s = s
        self._fin2.s = s/2
        self._fin3.s = s
  
    @property
    def Tf(self):
        return self._Tf
    @Tf.setter
    def Tf(self,Tf):
        self._Tf=Tf
        self._fin1.Tf = Tf
        self._fin2.Tf = Tf
        self._fin3.Tf = Tf

    @property
    def LCG(self):
        return self._LCG
    @LCG.setter
    def LCG(self,LCG):
        self._LCG=LCG
        self._L1 = LCG-self._Lnc
        self._L2 = self._Lbt-self._L1
        self._bodytube.L1 = self._L1
        self._nosecone.pos = vector(0,0,self._L1)
        self._fin1.pos = vector(self._Dr/2,0,-self._L2/4)    
        self._fin2.pos = vector(0,self._Dr/2,-self._L2)      
        self._fin3.pos = vector(-self._Dr/2,0,-self._L2/4)  
 
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self,pos):
        self._pos=pos
        self._frame1.pos=vector(pos)
        if self._CGtrail.CGtrail:
            self._CGtrail.append(pos=pos)
        if self._Tiptrail.Tiptrail:
            self._Tiptrail.append(pos=(vector(pos) + vector(self._frame1.axis)*(self._L1+self._Lnc)))

    @property
    def axis(self):
        return self._axis
    @axis.setter
    def axis(self,axis):
        self._axis=axis
        self._frame1.axis=vector(axis)
        if self._Tiptrail.Tiptrail:
            self._Tiptrail.append(pos=(vector(self._pos) + vector(self._frame1.axis)*(self._L1+self._Lnc)))

    @property
    def up(self):
        return self._up
    @up.setter
    def up(self,up):
        self._up=up
        self._frame1.up=vector(up)    


class axes(object):
    def __init__(self, frame=None, xlen=1.0, ylen=1.0, zlen=1.0, axiswidth=0.0, axis=(0,0,1), up=(1,0,0), labels=True,
                 xtext="X", ytext="Y", ztext="Z", tics=False, xtic=.1, color=color.white):

        self._frame1=newframe(frame=frame, axis=axis, up=up)
        self._frame2=newframe(frame=self._frame1, axis=(0,1,0), up=(0,0,1))
        self._xlen = xlen
        self._ylen = ylen
        self._zlen = zlen
        self._axiswidth = axiswidth
        self._axis=axis
        self._up=up
        self._xtext = xtext
        self._ytext = ytext
        self._ztext = ztext
        self._color = color
        
        self._axisx = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,0), (xlen,0,0)])
        self._axisy = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,0), (0,ylen,0)])
        self._axisz = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,0), (0,0,zlen)])

        if labels:
            self.xlabel=label(frame=self._frame2, pos=vector(xlen*1.05,0,0), text=xtext, box=0, opacity=0, color=self._color)
            self.ylabel=label(frame=self._frame2, pos=vector(0,ylen*1.05,0), text=ytext, box=0, opacity=0, color=self._color)
            self.zlabel=label(frame=self._frame2, pos=vector(0,0,zlen*1.05), text=ztext, box=0, opacity=0, color=self._color)

        if tics:
            n=int(xlen/xtic)
            for i in range(1,n+1):
                curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(i*xlen/n,0,0), (i*xlen/n,0,xlen/50)])
            
    @property
    def xlen(self):
        return xlen
    @xlen.setter
    def xlen(self,xlen):
        self._xlen=xlen
        self._axisx.pos = [(0,0,0), (xlen,0,0)]

    @property
    def ylen(self):
        return ylen
    @ylen.setter
    def ylen(self,ylen):
        self._ylen=ylen
        self._axisy.pos = [(0,0,0), (0,ylen,0)]

    @property
    def zlen(self):
        return zlen
    @zlen.setter
    def zlen(self,zlen):
        self._zlen=zlen
        self._axisz.pos = [(0,0,0), (0,0,zlen)]  

    @property
    def axiswidth(self):
        return axiswidth
    @axiswidth.setter
    def axiswidth(self,axiswidth):
        self._axiswidth=axiswidth
        self._axisx.radius = axiswidth
        self._axisy.radius = axiswidth
        self._axisz.radius = axiswidth

    @property
    # axis = z axis
    def axis(self):
        return axis
    @axis.setter
    def axis(self,axis):
        self._axis=axis
        self._frame1.axis = vector(axis)
        
    @property
    # up = x axis
    def up(self):
        return up
    @up.setter
    def up(self,up):
        self._up=up
        self._frame1.up = vector(up)

    @property
    def xtext(self):
        return xtext
    @xtext.setter
    def xtext(self,xtext):
        self._xtext=xtext
        self._xlabel.text = xtext

    @property
    def ytext(self):
        return ytext
    @ytext.setter
    def ytext(self,ytext):
        self._ytext=ytext
        self._ylabel.text = ytext

    @property
    def ztext(self):
        return ztext
    @ztext.setter
    def ztext(self,ztext):
        self._ztext=ztext
        self._zlabel.text = ztext    

class vectorx(object):
    def __init__(self, frame=None, vlen=1.0, axiswidth=0.0, pos=(0,0,0), axis=(0,0,1), up=(1,0,0), color=color.white, arrowsize=0.1):

        self._frame1=newframe(frame=frame, pos=pos, axis=axis, up=up)
        self._frame2=newframe(frame=self._frame1, axis=(0,1,0), up=(0,0,1))
        self._vlen = vlen
        self._axiswidth = axiswidth
        self._pos = pos
        self._axis=axis
        self._color=color
        self._arrowsize = arrowsize
        
                     
        self._vector1 = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,0), (0,0,vlen/2)])
        self._vector2 = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,0), (0,0,-vlen/2)])
        self._arrowhead1 = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,vlen/2), (self._arrowsize/1.4,0,vlen/2-self._arrowsize/1.4)])
        self._arrowhead2 = curve(frame=self._frame2, color=self._color, radius=axiswidth, pos=[(0,0,vlen/2), (-self._arrowsize/1.4,0,vlen/2-self._arrowsize/1.4)])

                    
    @property
    def vlen(self):
        return vlen
    @vlen.setter
    def vlen(self,vlen):
        self._vlen=vlen
        self._vector1.pos = [(0,0,0), (0,0,vlen/2)]
        self._vector2.pos = [(0,0,0), (0,0,-vlen/2)]
        self._arrowhead1.pos = [(0,0,vlen/2), (self._arrowsize/1.4,0,vlen/2-self._arrowsize/1.4)]
        self._arrowhead2.pos = [(0,0,vlen/2), (-self._arrowsize/1.4,0,vlen/2-self._arrowsize/1.4)]

            
    @property
    def axiswidth(self):
        return axiswidth
    @axiswidth.setter
    def axiswidth(self,axiswidth):
        self._axiswidth=axiswidth
        self._vector1.radius = axiswidth
        self._vector2.radius = axiswidth
        self._arrowhead1.radius = axiswidth
        self._arrowhead2.radius = axiswidth
    
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self,pos):
        self._pos=pos
        self._frame1.pos=vector(pos)
        
    @property
    # axis = z axis is the length of the vector
    def axis(self):
        return axis
    @axis.setter
    def axis(self,axis):
        self._axis=axis
        self._frame1.axis = vector(axis)

    @property
    # up = x axis which is the plane of the arrowheads
    def up(self):
        return up
    @up.setter
    def up(self,up):
        self._up=up
        self._frame1.up = vector(up)
        
    @property
    def color(self):
        return color
    @color.setter
    def color(self,color):
        self._color=color
        self._vector1.color = color
        self._vector2.color = color
        self._arrowhead1.color = color
        self._arrowhead2.color = color

    @property
    def arrowsize(self):
        return arrowsize
    @arrowsize.setter
    def arrowsize(self,arrowsize):
        self._arrowsize=arrowsize
        self._arrowhead1.pos = [(0,0,self._vlen/2), (self._arrowsize/1.4,0,self._vlen/2-self._arrowsize/1.4)]
        self._arrowhead2.pos = [(0,0,self._vlen/2), (-self._arrowsize/1.4,0,self._vlen/2-self._arrowsize/1.4)]
                    

# Self test code        
if __name__ == '__main__':

    r=2
    
    scene1=display(title="plane Orientation", width=600, height=600)
    scene1.range=(1.2,1.2,1.2)
    scene1.forward = (-1,-1,-.5)
    scene1.up=(0,0,1)
    scene1.select()
    axes1=axes(axiswidth=0.0, tics=True)
    f=frame()

    test = 3

    if test == 0:
        testbody=bodytube(frame=f)
        rate(r)
        testbody.Dr = .2
        rate(r)
        testbody.Lbt = 1
        rate(r)
        testbody.L1 = .4
        rate(r)
        testbody.axis=(0,1,0)
        rate(r)
        testbody.color=color.yellow
   
    if test == 1:
        testfin=fin(frame=f)
        rate(r)
        testfin.cr=.3
        rate(r)
        testfin.ct=.2
        rate(r)
        testfin.s=.3
        rate(r)
        testfin.Tf=.1
        rate(r)
        testfin.pos=(0,0,.6)
        rate(r)
        testfin.axis=(0,1,0)
        rate(r)
        testfin.up=(0,0,1)
        rate(r)
        testfin.color=color.yellow
   
    if test == 2:
        testnosecone=nosecone(frame=f)
        rate(r)
        testnosecone.Dr = .2
        rate(r)
        testnosecone.Lnc = .5
        rate(r)
        testnosecone.pos=(0,0,.3)
        rate(r)
        testnosecone.axis=(0,1,0)
        rate(r)
        testnosecone.color=color.yellow
        
    if test == 3:
        r1=plane()
        rate(r)
        #r1.pos = (.5,0,0)
        #rate(r)
        #r1.Lbt = 1
        #rate(r)

        #r1.Tf = .04
        rate(r)
        r1.axis = (0,1,0)
        rate(r)
        r1.axis = (0,0,1)
        rate(r)
        r1.axis = (-1,0,0)
        rate(r)
        r1.axis = (-1,0,-1)
        rate(r)
        r1.axis = (1,0,1)
 
    if test == 4:
        testvector=vectorx(frame=f)
        rate(r)
        testvector.vlen = 1.5
        rate(r)
        testvector.axis=(0,1,0)
        rate(r)
        testvector.arrowsize= .05
        rate(r)
        testvector.color = color.red
   
        
    if test == 5:
        rate(r)
        axes1.axis=(0,1,0)
        rate (r)
        axes1.xlen=.5
        rate (r)
        axes1.ylen=.5
        rate (r)
        axes1.zlen=.5