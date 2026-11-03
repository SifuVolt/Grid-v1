"""Small adapter around the RLBot v5 renderer for Party Cannon's debug API."""
from __future__ import annotations
from math import cos, pi, sin, sqrt
from .math3d import Vec3, Mat3x3

_COLORS={"white":(255,255,255,255),"red":(255,0,0,255),"lime":(0,255,0,255),"limegreen":(50,205,50,255),"cyan":(0,255,255,255),"yellow":(255,255,0,255)}
class ExtendedRenderer:
    def __init__(self,renderer,screen_width=1920,screen_height=1080):
        self.renderer=renderer;self.screen_width_scale=1/screen_width;self.screen_height_scale=1/screen_height;self.color="white"
    def begin_rendering(self):
        if self.renderer is not None:return self.renderer.begin_rendering()
    def end_rendering(self):
        if self.renderer is not None:return self.renderer.end_rendering()
    def is_rendering(self):
        return bool(self.renderer is not None and self.renderer.is_rendering())
    @property
    def can_render(self):
        return bool(self.renderer is not None and getattr(self.renderer, "can_render", True))
    def _color(self,color=None):
        name=str(color or self.color).lower()
        aliases={"limegreen":"lime", "green":"lime"}
        name=aliases.get(name,name)
        value=getattr(self.renderer,name,None)
        if value is not None:return value
        return _COLORS.get(name,_COLORS["white"])
    def _vec(self,v): return Vec3.from_value(v)
    def Text2D(self,text,upper_left,scale=1,color=None):
        if self.renderer is None:return
        p=self._vec(upper_left); self.renderer.draw_string_2d(str(text),p.x*self.screen_width_scale,p.y*self.screen_height_scale,scale,self._color(color))
    def Rect3D(self,pos,width,height,fill=True,color=None):
        fn=getattr(self.renderer,"draw_rect_3d",None) if self.renderer is not None else None
        if fn: fn(self._vec(pos),width*self.screen_width_scale,height*self.screen_height_scale,self._color(color))
    def Polyline3D(self,points,color=None):
        pts=[self._vec(p) for p in points]
        for a,b in zip(pts,pts[1:]):self.Line3D(a,b,color)
    def Circle(self,pos,normal,radius,color=None):
        offset=Vec3.from_value(normal).Cross(pos).Normalize()*radius;segments=int(radius**.69)+4;rot=Mat3x3.RotationFromAxis(Vec3.from_value(normal).Normalize(),2*pi/segments);pts=[]
        for _ in range(segments+1):offset=rot.Dot(offset);pts.append(Vec3.from_value(pos)+offset)
        self.Polyline3D(pts,color)
    def Cross(self,pos,size,color=None):
        p=self._vec(pos);h=size/2
        for axis in (Vec3.X,Vec3.Y,Vec3.Z):self.Line3D(p+axis*h,p-axis*h,color)
    def CrossAngled(self,pos,size,color=None):
        p=self._vec(pos);r=.5*size/sqrt(2)
        for a,b in [((r,r,r),(-r,-r,-r)),((r,r,-r),(-r,-r,r)),((r,-r,-r),(-r,r,r)),((r,-r,r),(-r,r,-r))]:self.Line3D(p+Vec3(*a),p+Vec3(*b),color)
    def Cube(self,pos,size,color=None):
        s=Vec3.from_value(size) if not isinstance(size,(int,float)) else Vec3(size,size,size);p=self._vec(pos);h=s/2
        vertices=[p+Vec3(x*h.x,y*h.y,z*h.z) for x,y,z in [(-1,-1,-1),(-1,-1,1),(1,-1,-1),(1,-1,1),(-1,1,-1),(-1,1,1),(1,1,-1),(1,1,1)]]
        for a,b in [(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]:self.Line3D(vertices[a],vertices[b],color)
    def OrientatedCube(self,pos,orientation,size,color=None):
        p=self._vec(pos);s=Vec3.from_value(size);rot=orientation.Transpose();h=s/2
        v=[p+rot.Dot(Vec3(x*h.x,y*h.y,z*h.z)) for x,y,z in [(-1,-1,-1),(-1,-1,1),(1,-1,-1),(1,-1,1),(-1,1,-1),(-1,1,1),(1,1,-1),(1,1,1)]]
        for a,b in [(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]:self.Line3D(v[a],v[b],color)
    def Octahedron(self,pos,size,color=None):
        p=self._vec(pos);h=size/2;v=[p+Vec3(h,0,0),p+Vec3(0,h,0),p+Vec3(-h,0,0),p+Vec3(0,-h,0),p+Vec3(0,0,h),p+Vec3(0,0,-h)]
        for a,b in [(0,1),(1,2),(2,3),(3,0),(0,4),(4,2),(2,5),(5,0),(1,4),(4,3),(3,5),(5,1)]:self.Line3D(v[a],v[b],color)
