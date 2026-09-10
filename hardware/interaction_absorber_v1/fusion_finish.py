"""Run in Fusion after importing cad/Airpot_V1_extended.step into a NEW document.
Creates native assembly relationships and exports a local F3D, no cloud save.
The source solids are imported BRep; dimensions are rebuilt with build_cad.py.
"""
import adsk.core
import adsk.fusion
import json
from pathlib import Path

ROOT = Path('/Users/shuqinzhu/Documents/FLS_Research/fls-cf-offboard-controller/hardware/interaction_absorber_v1')

def run(_context: str):
    app=adsk.core.Application.get()
    d=adsk.fusion.Design.cast(app.activeProduct)
    if d.rootComponent.occurrences.count!=1:
        raise RuntimeError('Expected a new document with one imported assembly.')
    wrapper=d.rootComponent.occurrences.item(0)
    c=wrapper.component
    if c.occurrences.count!=27 or c.asBuiltJoints.count:
        raise RuntimeError('Expected untouched 27-part V1 assembly.')
    d.designType=adsk.fusion.DesignTypes.ParametricDesignType
    manifest=json.loads((ROOT/'manifest.json').read_text())
    groups={p['name']:p['group'] for p in manifest['parts']}
    occ={o.component.name:o for o in c.occurrences}
    base=occ['P01_Base_rear_flange']; carriage=occ['P03_Carriage_contact_head']
    wrapper.isGrounded=True
    for o in occ.values():o.isGroundToParent=False
    base.isGroundToParent=True
    for name,o in occ.items():
        if o==base or o==carriage:continue
        parent=carriage if groups[name]=='moving' else base
        ji=c.asBuiltJoints.createInput(parent,o,None)
        ji.setAsRigidJointMotion()
        j=c.asBuiltJoints.add(ji)
        j.name='Rigid_'+name
    vertex=carriage.component.bRepBodies.item(0).vertices.item(0).createForAssemblyContext(carriage)
    geometry=adsk.fusion.JointGeometry.createByPoint(vertex)
    ji=c.asBuiltJoints.createInput(carriage,base,geometry)
    ji.setAsSliderJointMotion(adsk.fusion.JointDirections.XAxisJointDirection)
    joint=c.asBuiltJoints.add(ji)
    joint.name='Compression_22mm'
    motion=adsk.fusion.SliderJointMotion.cast(joint.jointMotion)
    limits=motion.slideLimits
    limits.minimumValue=-2.2; limits.maximumValue=0
    limits.isMinimumValueEnabled=True; limits.isMaximumValueEnabled=True
    # Fusion evaluates assembly movement when the enclosing command commits.
    # Drive/read verification therefore runs in subsequent tool invocations.
    for name,val in manifest['parameters'].items():
        d.userParameters.add('REF_'+name,adsk.core.ValueInput.createByString(str(val)+' mm'),'mm',
            'REFERENCE ONLY. Edit build_cad.py and regenerate to change geometry; does not resize imported bodies.')
    app.activeDocument.name='LightBender_Airpot_22mm_V1'
    app.activeViewport.fit()
    camera=app.activeViewport.camera
    camera.viewOrientation=adsk.core.ViewOrientations.IsoTopRightViewOrientation
    camera.isFitView=True
    app.activeViewport.camera=camera
    app.activeViewport.refresh()
    app.activeViewport.saveAsImageFile(str(ROOT/'preview'/'Fusion_assembly.png'),1600,1000)
    exporter=d.exportManager
    options=exporter.createFusionArchiveExportOptions(str(ROOT/'cad'/'LightBender_Airpot_22mm_V1.f3d'))
    if not exporter.execute(options):raise RuntimeError('F3D export failed')
    report={'native_joint_count':c.asBuiltJoints.count,'slider_joint':joint.name,'samples':[],
            'part_count':c.occurrences.count,'cloud_save':False,'geometry':'Imported BRep; native Fusion assembly joints'}
    (ROOT/'fusion_verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
