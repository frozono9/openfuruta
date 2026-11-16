"""Generate MJCF and URDF from inspected STEP meshes; never uses the old CAD."""
from pathlib import Path
import hashlib,json,copy
import xml.etree.ElementTree as E
import numpy as np
import trimesh

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'source/meshes'
CFG=json.loads((HERE/'config.json').read_text())
ARM_ORIGIN=np.array([-.0067,0,0])*1e-3
PIVOT=np.array([0,75,74.5])*1e-3-ARM_ORIGIN
GROUND_OFFSET=.0189127
GROUPS={'fixed':[0,3,4],'arm':[2,5,6],'pendulum':[7,8]}
COLORS={0:'.25 .3 .36 1',2:'.12 .45 .25 1',3:'.25 .32 .4 1',4:'.38 .45 .53 1',5:'.95 .36 .12 1',6:'.7 .72 .75 1',7:'.22 .6 .65 1',8:'.83 .62 .18 1'}
def fmt(a):return ' '.join(f'{v:.10g}' for v in np.asarray(a).ravel())
def elem(parent,tag,**kw):return E.SubElement(parent,tag,{k:str(v) for k,v in kw.items()})
def tensor6(I):return [I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]
def compound(parts):
    total=sum(m for _,m in parts)
    c=sum(mesh.center_mass*m for mesh,m in parts)/total
    I=np.zeros((3,3))
    for mesh,m in parts:
        if mesh.volume<=0:raise ValueError('Nonpositive mesh volume')
        d=mesh.center_mass-c
        I+=mesh.moment_inertia*(m/mesh.volume)+m*((d@d)*np.eye(3)-np.outer(d,d))
    if np.linalg.eigvalsh(I).min()<=0:raise ValueError('Invalid inertia')
    return total,c,I

def main():
    (HERE/'assets').mkdir(exist_ok=True)
    components=json.loads((SOURCE/'components.json').read_text())
    expected={0:'17HE15-1504S v1',2:'AS5600 v1',3:'MotorHolder',4:'Bottom',5:'Arm',6:'Bearing',7:'PendulumArm',8:'50CentCoin'}
    for i,name in expected.items():
        if components[i]['definition']!=name:raise ValueError('STEP component order changed; update the explicit link mapping before rebuilding')
    meshes={};report={}
    for name,ids in GROUPS.items():
        for i in ids:
            mesh=trimesh.load_mesh(SOURCE/f'{i:02d}.stl')
            mesh.apply_scale(.001);mesh.apply_translation(-ARM_ORIGIN)
            if name=='pendulum':mesh.apply_translation(-PIVOT)
            mesh.export(HERE/'assets'/f'{i:02d}.stl');meshes[i]=mesh
    mass=CFG['mass_kg'];bearing_inner=meshes[6].copy();bearing_inner.apply_translation(-PIVOT)
    inertias={
      'arm':compound([(meshes[5],mass['arm']),(meshes[2],mass['sensor']),(meshes[6],mass['bearing_outer'])]),
      'pendulum':compound([(meshes[7],mass['pendulum_print']),(meshes[8],mass['coin']),(bearing_inner,mass['bearing_inner'])])}
    root=E.Element('mujoco',model='ALatorre_Furuta_single_encoder')
    elem(root,'compiler',angle='radian',meshdir='assets',inertiafromgeom='false')
    elem(root,'option',timestep=CFG['physics_dt'],gravity='0 0 -9.81',integrator='implicitfast')
    vis=elem(root,'visual');elem(vis,'global',offwidth='960',offheight='720')
    asset=elem(root,'asset')
    for i in meshes:elem(asset,'mesh',name=f'part{i}',file=f'{i:02d}.stl')
    world=elem(root,'worldbody')
    elem(world,'light',pos='.2 -.2 .5',dir='-.2 .2 -.5',diffuse='.8 .8 .8')
    elem(world,'geom',name='floor',type='plane',size='1 1 .01',rgba='.92 .94 .96 1',contype='0',conaffinity='0')
    elem(world,'camera',name='overview',pos='.29 .34 .23',xyaxes='-.761 .649 0 -.25 -.293 .923')
    fixed=elem(world,'body',name='base',pos=f'0 0 {GROUND_OFFSET}')
    arm=elem(fixed,'body',name='arm')
    elem(arm,'joint',name='arm_joint',type='hinge',axis='0 0 1',limited='true',range=fmt(np.deg2rad([-CFG['hard_stop_deg'],CFG['hard_stop_deg']])),damping=CFG['arm_damping_nm_s'],armature=CFG['rotor_inertia_kg_m2'])
    pend=elem(arm,'body',name='pendulum',pos=fmt(PIVOT))
    elem(pend,'joint',name='pendulum_joint',type='hinge',axis='0 1 0',limited='false',damping=CFG['hinge_damping_nm_s'],frictionloss=CFG['hinge_friction_nm'])
    bodies={'fixed':fixed,'arm':arm,'pendulum':pend}
    for name,ids in GROUPS.items():
        b=bodies[name]
        if name in inertias:
            m,c,I=inertias[name];elem(b,'inertial',pos=fmt(c),mass=m,fullinertia=fmt(tensor6(I)))
            report[name]={'mass_kg':m,'com_m':c.tolist(),'inertia_kg_m2':I.tolist()}
        for i in ids:elem(b,'geom',name=f'part{i}',type='mesh',mesh=f'part{i}',rgba=COLORS[i],contype='0',conaffinity='0',group='1')
    elem(pend,'site',name='coin_center',pos=fmt(np.array([0,75,-.5])*.001-ARM_ORIGIN-PIVOT),size='.002',rgba='1 0 0 1')
    actuators=elem(root,'actuator');elem(actuators,'motor',name='motor_torque',joint='arm_joint',gear='1',ctrllimited='true',ctrlrange='-1 1')
    E.indent(root);E.ElementTree(root).write(HERE/'furuta.xml',encoding='unicode')
    # URDF provides geometry/inertia only; motor and sensor dynamics live in env.py.
    robot=E.Element('robot',name='ALatorre_Furuta')
    elem(robot,'link',name='world')
    for name,ids in GROUPS.items():
        link=elem(robot,'link',name='base' if name=='fixed' else name)
        if name in inertias:
            m,c,I=inertias[name];inert=elem(link,'inertial');elem(inert,'origin',xyz=fmt(c));elem(inert,'mass',value=m)
            elem(inert,'inertia',ixx=I[0,0],iyy=I[1,1],izz=I[2,2],ixy=I[0,1],ixz=I[0,2],iyz=I[1,2])
        for i in ids:
            v=elem(link,'visual');g=elem(v,'geometry');elem(g,'mesh',filename=f'assets/{i:02d}.stl');mat=elem(v,'material',name=f'color{i}');elem(mat,'color',rgba=COLORS[i])
    for name,typ,parent,child,pos,axis in [('mount','fixed','world','base',[0,0,GROUND_OFFSET],None),('arm_joint','revolute','base','arm',[0,0,0],[0,0,1]),('pendulum_joint','continuous','arm','pendulum',PIVOT,[0,1,0])]:
        j=elem(robot,'joint',name=name,type=typ);elem(j,'parent',link=parent);elem(j,'child',link=child);elem(j,'origin',xyz=fmt(pos))
        if axis is not None:elem(j,'axis',xyz=fmt(axis))
        if typ=='revolute':elem(j,'limit',lower=-np.deg2rad(CFG['hard_stop_deg']),upper=np.deg2rad(CFG['hard_stop_deg']),effort=CFG['motor_peak_torque_nm'],velocity=CFG['max_speed_rad_s'])
    E.indent(robot);E.ElementTree(robot).write(HERE/'furuta.urdf',encoding='unicode')
    report.update({'source_sha256':hashlib.sha256((HERE.parent/'hardware/cad/OpenFuruta.step').read_bytes()).hexdigest(),'hinge_in_arm_frame_m':PIVOT.tolist(),'assumptions':CFG['notes'],'contacts':'Visual meshes have no contact. Joint limit only; not a full collision validation.'})
    (HERE/'model_report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
