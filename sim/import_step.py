from pathlib import Path
import json
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
import numpy as np
import trimesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder

import argparse
here = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description="Export assembly meshes from the OpenFuruta STEP")
parser.add_argument("step", nargs="?", type=Path, default=here.parent / "hardware/cad/OpenFuruta.step")
args = parser.parse_args()
OUT = here / "source/meshes"
OUT.mkdir(parents=True, exist_ok=True)
source = args.step.resolve()
doc=TDocStd_Document(TCollection_ExtendedString('inspection'))
r=STEPCAFControl_Reader();r.SetNameMode(True);
if r.ReadFile(str(source)) != IFSelect_RetDone or not r.Transfer(doc):
    raise RuntimeError(f"Cannot import {source}")
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots=TDF_LabelSequence();st.GetFreeShapes(roots)
rows=[];scene=trimesh.Scene()
colors=['#52677d','#85b4c5','#ecab4c','#87929d','#c6d8e6','#df815a','#a8b8bf','#99b781','#dfc263']
def name(label):
    a=TDataStd_Name()
    return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),a) else 'unnamed'
def walk(label,loc,path):
    nm=name(label);loc=loc.Multiplied(st.GetLocation_s(label))
    ref=TDF_Label()
    if st.IsReference_s(label):st.GetReferredShape_s(label,ref)
    else:ref=label
    parts=TDF_LabelSequence()
    if st.GetComponents_s(ref,parts):
        for i in range(1,parts.Length()+1):walk(parts.Value(i),loc,path+[nm])
        return
    shape=st.GetShape_s(ref).Moved(loc)
    box=Bnd_Box();BRepBndLib.Add_s(shape,box)
    props=GProp_GProps();BRepGProp.VolumeProperties_s(shape,props)
    BRepMesh_IncrementalMesh(shape,.15,False,.15,True)
    vs=[];fs=[];faces=[];ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        face=TopoDS.Face_s(ex.Current());faces.append(face);fl=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(face,fl)
        if tri is not None:
            off=len(vs)
            for n in range(1,tri.NbNodes()+1):
                p=tri.Node(n).Transformed(fl.Transformation());vs.append((p.X(),p.Y(),p.Z()))
            for n in range(1,tri.NbTriangles()+1):
                ids=[off+k-1 for k in tri.Triangle(n).Get()]
                if face.Orientation()==TopAbs_REVERSED:ids.reverse()
                fs.append(ids)
        ex.Next()
    ex=TopExp_Explorer(shape,TopAbs_SOLID);ns=0
    while ex.More():ns+=1;ex.Next()
    mesh=trimesh.Trimesh(vertices=vs,faces=fs,process=True)
    idx=len(rows);file=f'{idx:02d}.stl';mesh.export(OUT/file)
    color=colors[idx%len(colors)];mesh.visual.face_colors=trimesh.visual.color.hex_to_rgba(color);scene.add_geometry(mesh,node_name=f'{idx}_{nm}')
    cylinders=[]
    for f in faces:
        surf=BRepAdaptor_Surface(f)
        if surf.GetType()==GeomAbs_Cylinder:
            c=surf.Cylinder();p=c.Location();d=c.Axis().Direction()
            item={'radius':round(c.Radius(),4),'point':[round(x,4) for x in (p.X(),p.Y(),p.Z())],'axis':[round(x,4) for x in (d.X(),d.Y(),d.Z())]}
            if item not in cylinders:cylinders.append(item)
    rows.append({'index':idx,'name':nm,'definition':name(ref),'path':path,'file':file,'color':color,'solids':ns,'volume_mm3':props.Mass(),'center_mm':[props.CentreOfMass().X(),props.CentreOfMass().Y(),props.CentreOfMass().Z()],'bounds_mm':[list(box.Get()[:3]),list(box.Get()[3:])],'cylinders':cylinders})
for i in range(1,roots.Length()+1):walk(roots.Value(i),TopLoc_Location(),[])
(OUT/'components.json').write_text(json.dumps(rows,indent=2))
scene.export(OUT/'assembly.glb')
for row in rows: print(row['index'],row['name'],row['definition'],'solids',row['solids'],'bounds',np.round(row['bounds_mm'],2).tolist())
