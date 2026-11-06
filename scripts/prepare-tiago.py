"""Generate an auditable PAL-derived 20-DoF planar-supported TIAGo Pro model.
Only Python stdlib is needed. Original inertial entries remain unchanged.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib, json

root=Path(__file__).resolve().parents[1]
folder=root/'src/vendor/tiago-pro'
source=folder/'original.xml'
tree=ET.parse(source); model=tree.getroot()
model.set('model','PAL_TIAGo_Pro_planar_20dof')
model.find('compiler').attrib.update({'inertiafromgeom':'false'})
model.find('compiler').attrib.pop('meshdir',None)
removed=0
for parent in model.iter():
    for child in list(parent):
        if child.tag=='mesh' or (child.tag=='geom' and child.get('mesh')):
            parent.remove(child); removed+=1
model.insert(1,ET.Element('option',{'timestep':'0.005','integrator':'implicitfast','gravity':'0 0 -9.81','iterations':'50'}))
body=model.find('worldbody/body')
for i,(name,kind,axis) in enumerate([('base_x','slide','1 0 0'),('base_y','slide','0 1 0'),('base_yaw','hinge','0 0 1')]):
    body.insert(i,ET.Element('joint',{'name':name,'type':kind,'axis':axis,'limited':'false','damping':'2' if i<2 else '1'}))
extras=[('torso_lift_link','torso_lift_joint','slide','0 0 1','0 0.35','-2000 2000','12'),('head_1_link','head_1_joint','hinge','0 0 1','-1.309 1.309','-5.197 5.197','0.5'),('head_2_link','head_2_joint','hinge','0 0 -1','-1.0472 0.34907','-2.77 2.77','0.5')]
for b,n,t,a,r,f,d in extras:
    target=next(x for x in model.iter('body') if x.get('name')==b)
    target.insert(0,ET.Element('joint',{'name':n,'type':t,'axis':a,'range':r,'actuatorfrcrange':f,'damping':d}))
# Preserve published inertia. Arm damping and rotor inertia are declared experiment values.
for j in body.iter('joint'):
    if j.get('name','').startswith('arm_'):j.attrib.update({'damping':'0.25','armature':'0.01'})
act=ET.SubElement(model,'actuator')
for j in body.iter('joint'):
    n=j.get('name'); force=j.get('actuatorfrcrange') or ('-250 250' if n in ['base_x','base_y'] else '-100 100')
    ET.SubElement(act,'motor',{'name':n+'_motor','joint':n,'gear':'1','ctrllimited':'true','ctrlrange':force})
ET.indent(tree,space='  '); destination=folder/'planar20.xml'; destination.write_text(ET.tostring(model,encoding='unicode'),encoding='utf-8',newline='\n')
original=ET.parse(source).getroot();assert [x.attrib for x in original.iter('inertial')]==[x.attrib for x in model.iter('inertial')]
info={'schema':'pal-tiago-model-provenance-v1','sourceRepository':'https://github.com/pal-robotics/pal_mjlab','sourceCommit':'952abb83a590d0e771cfd8e570517ba054182196','sourcePath':'src/pal_mjlab/robots/pal_tiago_pro/xmls/tiago_pro.xml','headSourceCommit':'a10e252dc55968c6ac91d2a84eb4626a4811bc78','sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,destination]},'inertialEntriesPreserved':len(list(model.iter('inertial'))),'coordinates':20,'removedMeshEntries':removed,'modifications':['Remove mesh visuals, retain all published explicit inertial entries and primitive collision geometry.','Add planar x/y/yaw support: wheel-ground/roller dynamics and chassis roll/pitch are not modeled.','Restore torso slide and head pan/tilt joints; fixed gripper fingers remain fixed.','Add bounded ideal generalized-force actuators, damping and armature as declared experimental parameters.','Base inputs are chassis forces/yaw moment, not measured factory wheel torque commands.'],'calibration':'Published vendor simulation model, not identified hardware dynamics.'}
(folder/'provenance.json').write_text(json.dumps(info,indent=2)+'\n',encoding='utf-8',newline='\n')
print(json.dumps(info))
