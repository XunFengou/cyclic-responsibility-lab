"""Independent Python MuJoCo audit of the same generated MJCF. Not hardware QA."""
from pathlib import Path
import json, hashlib
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
root=Path(__file__).resolve().parents[1]; folder=root/'src/vendor/tiago-pro'
original=ET.parse(folder/'original.xml').getroot(); generated=ET.parse(folder/'planar20.xml').getroot()
assert [x.attrib for x in original.iter('inertial')]==[x.attrib for x in generated.iter('inertial')]
m=mujoco.MjModel.from_xml_path(str(folder/'planar20.xml'));d=mujoco.MjData(m)
d.qpos[3]=.15
for side,sign in [('left',1),('right',-1)]:
    for j,v in {1:sign*.3578,2:-1.8266,3:sign*.4698,4:-2.3409,6:-1.2006}.items():d.qpos[m.joint(f'arm_{side}_{j}_joint').qposadr]=v
mujoco.mj_forward(m,d); matrix=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,matrix,d.qM)
np.linalg.cholesky(matrix);assert m.nv==20 and m.nu==20
coupling=float(np.max(np.abs(matrix[:3,6:])));assert coupling>1e-3
report={'status':'passed','engine':mujoco.__version__,'nv':m.nv,'nu':m.nu,'preservedInertialEntries':len(list(original.iter('inertial'))),'publishedModelMassKg':float(np.sum(m.body_mass)),'minimumInertiaEigenvalue':float(np.linalg.eigvalsh(matrix)[0]),'maximumBaseUpperBodyInertiaCoupling':coupling,'modelSha256':hashlib.sha256((folder/'planar20.xml').read_bytes()).hexdigest(),'scope':'Independent native Python compilation and inertia audit; not factory parameter identification.'}
(root/'test-results').mkdir(exist_ok=True);(root/'test-results/tiago_python_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
