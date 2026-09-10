"""Mesh checks and a geometry-derived layout sheet, not a toleranced drawing."""
import json
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import build_cad as cad

ROOT=Path(__file__).resolve().parent

def main():
    checks=[]
    for path in sorted((ROOT/'stl').glob('*.stl')):
        mesh=trimesh.load_mesh(path,process=True)
        check=dict(file=path.name,watertight=bool(mesh.is_watertight),
                   winding_consistent=bool(mesh.is_winding_consistent),volume_mm3=float(mesh.volume),
                   minimum_z_mm=float(mesh.bounds[0,2]))
        if not check['watertight'] or not check['winding_consistent'] or check['volume_mm3']<=0:
            raise RuntimeError(str(check))
        checks.append(check)
    overlaps=[]
    for i,a in enumerate(cad.parts):
        for b in cad.parts[i+1:]:
            av,bv=a['shape'].val(),b['shape'].val()
            ab,bb=av.BoundingBox(),bv.BoundingBox()
            if ab.xmax<=bb.xmin+1e-7 or ab.xmin>=bb.xmax-1e-7 or ab.ymax<=bb.ymin+1e-7 or ab.ymin>=bb.ymax-1e-7 or ab.zmax<=bb.zmin+1e-7 or ab.zmin>=bb.zmax-1e-7:continue
            v=av.intersect(bv).Volume()
            if v>1e-5:overlaps.append(dict(a=a['name'],b=b['name'],overlap_mm3=v))
    (ROOT/'mesh_and_assembly_qa.json').write_text(json.dumps(dict(stl=checks,
         assembled_all_pair_interferences=overlaps),indent=2))
    if overlaps:raise RuntimeError('Unexpected assembled overlap: '+str(overlaps))
    fig=plt.figure(figsize=(14,9),facecolor='#f6f8fa')
    for i,(s,label) in enumerate([(0,'EXTENDED | 0 mm compression'),(22,'COMPRESSED | 22 mm compression')]):
        ax=fig.add_subplot(2,1,i+1,projection='3d',computed_zorder=True)
        ax.set_facecolor('#f6f8fa')
        for part in cad.parts:
            verts,faces=part['shape'].val().tessellate(.10,.20)
            v=np.array([[p.x,p.y,p.z] for p in verts])
            if part['group']=='moving':v[:,0]-=s
            poly=Poly3DCollection(v[np.array(faces)],facecolors=[part['color']],
                edgecolors=[(*part['color'],.15)],linewidths=.06,zsort='average')
            ax.add_collection3d(poly)
        ax.set_xlim(-12,95);ax.set_ylim(-26,26);ax.set_zlim(0,25)
        ax.set_box_aspect((107,52,25),zoom=2.0);ax.set_proj_type('ortho');ax.view_init(elev=24,azim=-61)
        ax.set_axis_off();ax.set_title(label,loc='left',fontweight='bold',fontsize=13,pad=-5,color='#253747')
    fig.suptitle('LightBender / AIRPOT GUIDED ABSORBER V1',fontsize=20,fontweight='bold',x=.075,ha='left',y=.98,color='#253747')
    fig.text(.075,.935,'99.1 x 46 x 24 mm envelope  |  22 mm working travel  |  ~34-37 g mechanical estimate',fontsize=11,color='#536576')
    fig.text(.075,.065,'Blue: purchased air dashpot   |   Green: moving carriage   |   Orange: TPU pads   |   Gray: stationary frame',fontsize=11,color='#253747')
    fig.text(.075,.035,'MANUAL DAMPING ADJUSTMENT.  1 N is a test target, NOT a force cap.  Prototype: bench-test before flight.',fontsize=10,color='#9c4b21')
    fig.subplots_adjust(top=.9,bottom=.11,left=.06,right=.96,hspace=.04)
    fig.savefig(ROOT/'preview'/'Layout_extended_compressed.png',dpi=160)
    plt.close(fig)
    print(json.dumps({'stl_count':len(checks),'all_watertight':True,'all_pair_interferences':overlaps}))

if __name__=='__main__':main()
