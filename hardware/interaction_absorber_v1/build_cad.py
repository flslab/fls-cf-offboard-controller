"""Airpot guided absorber V1 — millimetres, CadQuery 2.6.1.

Concept/prototype geometry, not a flight-qualified design. Purchased parts are
dimensioned interface/envelope models, not manufacturer production CAD.
Edit P and regenerate. Fusion imports have editable BRep bodies, not a linked
parametric timeline. No servo or automatic force control is implemented.
"""
from pathlib import Path
import json
import math
import cadquery as cq

OUT = Path(__file__).resolve().parent
P = dict(stroke_mm=22.0, airpot_stroke_mm=25.4, rail_spacing_mm=32.0,
         shaft_d_mm=2.0, bearing_id_mm=2.034, bearing_od_mm=3.5,
         bearing_len_mm=3.0, housing_bore_mm=3.5, shaft_axis_z_mm=12.0,
         base_t_mm=2.5, carriage_length_mm=18.0)
margin = (P['airpot_stroke_mm'] - P['stroke_mm']) / 2
q = P['airpot_stroke_mm'] + .860*25.4 + margin + P['stroke_mm']
z = P['shaft_axis_z_mm']; ys = [-P['rail_spacing_mm']/2, P['rail_spacing_mm']/2]
cb = q-12; cf = q+6
rs = cb-P['stroke_mm']-2
front = cf+2; fend = front+4
parts = []

def box(x0,x1,y0,y1,z0,z1):
    return cq.Workplane('XY').box(x1-x0,y1-y0,z1-z0).translate(((x0+x1)/2,(y0+y1)/2,(z0+z1)/2))

def cyl(x0,x1,y,z0,r,ri=0):
    w=cq.Workplane('YZ',origin=(x0,y,z0)).circle(r)
    if ri: w=w.circle(ri)
    return w.extrude(x1-x0)

def zcyl(x,y,z0,z1,r):
    return cq.Workplane('XY',origin=(x,y,z0)).circle(r).extrude(z1-z0)

def xhex(x0,x1,y,z0,af,ri):
    return cq.Workplane('YZ',origin=(x0,y,z0)).polygon(6,af/math.cos(math.pi/6)).circle(ri).extrude(x1-x0)

def add(name,shape,group='fixed',kind='purchase',color=(.6,.65,.72),note=''):
    shape=shape.clean()
    if not shape.val().isValid(): raise RuntimeError('Invalid BRep: '+name)
    if len(shape.solids().vals())!=1: raise RuntimeError('Not one solid: '+name)
    parts.append(dict(name=name,shape=shape,group=group,kind=kind,color=color,note=note))

# Skeletal bed, rear flange, and compression-stop towers: one printed part.
base=box(-2.5,1,-23,23,0,23)
for y in ys:
    base=base.union(box(0,fend+1,y-4,y+4,0,2.5))
    base=base.union(box(rs-2,rs,y-4,y+4,2.5,17))
    base=base.union(cyl(rs,rs+1.2,y,z,2.3,1.2))  # rigid backup, 0.8 mm beyond nominal stop
    base=base.cut(cyl(-1.5,1.1,y,z,1.02))  # blind rear shaft sockets
    base=base.cut(cyl(rs-2.1,rs+.1,y,z,1.15))
    for x in [9,70]:
        # M2 mounting slots: total length 6 mm, width 2.5 mm.
        slot=cq.Workplane('XY',origin=(x,y,-.1)).slot2D(6,2.5).extrude(2.7)
        base=base.cut(slot)
        countersink=(cq.Workplane('XY',origin=(x,y,1.5)).slot2D(6,2.5)
            .workplane(offset=1.01).slot2D(8.02,4.52).loft())
        base=base.cut(countersink)  # flush M2 countersunk screw; no washer above bed
base=base.union(box(front,fend+1,-23,23,0,4))
base=base.cut(cyl(-2.6,1.1,0,z,5.0))
for y in [-20,20]:
    base=base.cut(zcyl(front+2,y,-.1,4.1,1.1))
    base=base.cut(cq.Workplane('XY',origin=(front+2,y,-.1)).polygon(6,4.8).extrude(1.75))
# Web top above the 2.5 mm clamped rear flange is removed on the cylinder side.
base=base.cut(box(0,1.1,-10,10,2.5,23.1))
add('P01_Base_rear_flange',base,kind='PETG',color=(.23,.29,.36))

# Removable front bridge captures both shaft tips. M2x16 screws + captured nuts.
bridge=box(front,fend,-23,23,4,5.5)
for y in ys:
    lo,hi=(10,23) if y>0 else (-23,-10)
    bridge=bridge.union(box(front,fend,lo,hi,4,18))
    bridge=bridge.union(cyl(front-1.2,front,y,z,2.3,1.2))
    bridge=bridge.cut(cyl(front-.1,fend-1,y,z,1.02))
for y in [-20,20]:
    bridge=bridge.cut(zcyl(front+2,y,3.9,18.1,1.1))
    bridge=bridge.cut(zcyl(front+2,y,16,18.1,2.05))
add('P02_Front_bridge',bridge,kind='PETG',color=(.23,.29,.36))

# Guided carriage. NF rod flange is 1.5 mm thick so its supplied small nut fits.
car=box(q-6,cf,-19,19,4.5,7.5)
for y in ys:
    car=car.union(box(cb,cf,y-3,y+3,7,17))
    car=car.cut(cyl(cb-.1,cf+.1,y,z,1.2))
    car=car.cut(cyl(cb-.1,cb+3.5,y,z,P['housing_bore_mm']/2))
    car=car.cut(cyl(cf-3.5,cf+.1,y,z,P['housing_bore_mm']/2))
car=car.union(box(q,q+1.5,-5,5,7,21))
car=car.cut(cyl(q-.1,q+1.6,0,z,1.25))
car=car.union(box(q+1.4,q+16,-8,8,17,21))
head=box(q+16,q+19,-10,10,6,24).edges('|X').fillet(2)
car=car.union(head)
add('P03_Carriage_contact_head',car,'moving','PETG',(.12,.66,.55))
pad=box(q+19,q+20.5,-9,9,7,23).edges('|X').fillet(3)
add('P04_Contact_pad',pad,'moving','TPU',(.96,.49,.16),'Bond to head; TPU is not a force limiter.')

for side,y in zip(['L','R'],ys):
    add('B01_Shaft_'+side,cyl(-1.5,fend-1,y,z,1),color=(.75,.79,.83))
    for i,x in enumerate([cb+.25,cf-3.25]):
        add(f'B02_igus_GSM020303_{side}{i+1}',cyl(x,x+3,y,z,1.75,P['bearing_id_mm']/2),
            'moving','purchase',(.18,.16,.12),'2 x 3.5 x 3 mm; housing must be finish-reamed.')
    for label,x in [('rear',rs),('front',cf)]:
        add(f'P05_Bumper_{side}_{label}',cyl(x,x+2,y,z,3.5,2.45),kind='TPU',color=(.96,.49,.16))

# Commercial Airpot envelope: mounting shoulder at X=0, NF shoulder at X=q.
bodylen=P['airpot_stroke_mm']+.437*25.4
capstart=bodylen-.140*25.4
glass=cyl(1.27,capstart,0,z,.311*25.4/2,.220*25.4/2)
add('B03_Airpot_glass_envelope',glass,color=(.43,.75,.90),note='Borosilicate OD max; internal model simplified.')
airhead=cyl(-.225*25.4,0,0,z,9.525/2,1).union(cyl(0,1.27,0,z,5.08,1))
add('B04_Airpot_head_thread_envelope',airhead,color=(.17,.22,.28))
add('B05_Airpot_front_cap',cyl(capstart,bodylen,0,z,5.08,4.572/2),color=(.17,.22,.28))
add('B06_Airpot_adjuster_envelope',cyl(-7.62,-5.715,0,z,2.0),color=(.80,.59,.25),note='Keep rear tool access; no servo adapter.')
add('B07_Airpot_mount_washer',cyl(-3.092,-2.5,0,z,6.5,4.8),color=(.8,.8,.8))
add('B08_Airpot_mount_nut',xhex(-5.4796,-3.092,0,z,12.7,4.8),color=(.70,.72,.74))
# Piston/rod interior dimensions are conservative clearance envelopes, not OEM manufacturing data.
piston=cyl(27,31.5,0,z,2.70)
rod=piston.union(cyl(30,q-4.826,0,z,.058*25.4/2))
rod=rod.union(cyl(q-4.826,q-1,0,z,1.6)).union(xhex(q-1,q,0,z,4.7625,.5))
rod=rod.union(cyl(q,q+.163*25.4,0,z,2.1844/2))
add('B09_Airpot_piston_NF_rod',rod,'moving',color=(.70,.72,.75))
add('B10_NF_lockwasher',cyl(q+1.5,q+2.008,0,z,2.3,1.11),'moving',color=(.75,.75,.75))
add('B11_NF_2_64_nut',xhex(q+2.008,q+3.5828,0,z,4.7625,1.11),'moving',color=(.76,.64,.34))
for side,y in [('L',-20),('R',20)]:
    bolt=zcyl(front+2,y,0,16,1).union(zcyl(front+2,y,16,18,1.9))
    bolt=bolt.cut(cq.Workplane('XY',origin=(front+2,y,17)).polygon(6,1.75).extrude(1.1))
    add('B12_M2x16_'+side,bolt,color=(.5,.54,.58))
    nut=cq.Workplane('XY',origin=(front+2,y,0)).polygon(6,4/math.cos(math.pi/6)).circle(1.05).extrude(1.6)
    add('B13_M2_nut_'+side,nut,color=(.52,.55,.58))

def assembly(compression=0, exploded=False):
    a=cq.Assembly(name='LightBender_Airpot_22mm_V1')
    for p in parts:
        dx=-compression if p['group']=='moving' else 0
        dz=0
        if exploded:
            if p['group']=='moving': dz=25
            elif p['name'].startswith('B0') and 'Shaft' not in p['name']: dz=12
            if p['name']=='P02_Front_bridge': dx=15
        a.add(p['shape'],name=p['name'],color=cq.Color(*p['color']),loc=cq.Location(cq.Vector(dx,0,dz)))
    return a

def main():
    for sub in ['cad','stl','parts_step','preview']:(OUT/sub).mkdir(exist_ok=True)
    manifest=[]
    for p in parts:
        cq.exporters.export(p['shape'],str(OUT/'parts_step'/(p['name']+'.step')))
        bb=p['shape'].val().BoundingBox()
        entry={k:v for k,v in p.items() if k!='shape'}
        entry.update(volume_mm3=p['shape'].val().Volume(),bbox_mm=[bb.xlen,bb.ylen,bb.zlen])
        manifest.append(entry)
        if p['kind'] in ('PETG','TPU'):
            # Rotate bore-axis-X printed parts onto their broad YZ faces.
            # Base prints flat. Carriage prints on its outer contact-head face.
            shape=p['shape']
            if p['name'].startswith(('P03','P04','P05')):
                shape=shape.rotate((0,0,0),(0,1,0),90)
            b=shape.val().BoundingBox()
            shape=shape.translate((-b.xmin,-b.ymin,-b.zmin))
            cq.exporters.export(shape,str(OUT/'stl'/(p['name']+'.stl')),tolerance=.03,angularTolerance=.1)
    assembly().save(str(OUT/'cad'/'Airpot_V1_extended.step'))
    assembly(P['stroke_mm']).save(str(OUT/'cad'/'Airpot_V1_compressed.step'))
    assembly(exploded=True).save(str(OUT/'cad'/'Airpot_V1_exploded.step'))
    result=dict(parameters=P,datums_mm=dict(airpot_mount_x=0,rod_shoulder_extended=q,
        rod_shoulder_compressed=q-P['stroke_mm'],usable_stroke=P['stroke_mm'],
        nominal_end_margin=margin,rail_cut_length=fend+.5,
        rear_bumper_start=rs,front_bridge_start=front),parts=manifest)
    (OUT/'manifest.json').write_text(json.dumps(result,indent=2))
    # Sweep each moving part against every fixed part at 23 evenly spaced positions.
    # Mating faces and nominal bearing press fits have no volumetric overlap.
    interference=[]
    fixed=[p for p in parts if p['group']=='fixed']
    moving=[p for p in parts if p['group']=='moving']
    for s in range(23):
        for m in moving:
            ms=m['shape'].translate((-float(s),0,0)).val()
            for f in fixed:
                fs=f['shape'].val()
                if not ms.BoundingBox().isInside(fs.BoundingBox()):
                    # Bounding-box intersection prefilter; OCCT common is definitive.
                    a,b=ms.BoundingBox(),fs.BoundingBox()
                    if any([a.xmax<=b.xmin+1e-7,a.xmin>=b.xmax-1e-7,a.ymax<=b.ymin+1e-7,
                            a.ymin>=b.ymax-1e-7,a.zmax<=b.zmin+1e-7,a.zmin>=b.zmax-1e-7]):continue
                v=ms.intersect(fs).Volume()
                if v>1e-5:interference.append(dict(compression_mm=s,moving=m['name'],fixed=f['name'],overlap_mm3=v))
    report=dict(all_solids_valid=True,part_count=len(parts),sweep_positions_mm=list(range(23)),
                moving_fixed_interferences=interference,
                printed_solid_material_mass_g=sum(p['shape'].val().Volume()*({'PETG':.00127,'TPU':.00121}.get(p['kind'],0)) for p in parts),
                limitations=['Discrete geometric sweep, not continuous collision proof or FEA.',
                             'OEM internal geometry simplified; no wires, tolerance stack or elastic deformation simulated.',
                             '1 N is a test target, not a force cap. TPU stops can transmit much more.',
                             'No electronic regulation, automatic return, or cage-specific interface in V1.'])
    (OUT/'verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    if interference:raise RuntimeError('Moving/fixed collision; inspect verification.json')

if __name__=='__main__':main()
