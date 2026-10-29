#pragma once
#include <stdbool.h>
#include <math.h>

// The control origin is IMU + the constant neutral-world offset r. Thus a
// level vehicle retains the historical marker-coordinate hover height, while
// all translational derivatives belong to the IMU point, never the COM.
// q is [w,x,y,z]; J is for a RIGHT/body small attitude error.
static inline bool markerGeometry(const float q[4],const float r[3],
    float offset[3],float J[3][3]) {
  float n=0;
  for(unsigned i=0;i<4;i++){if(!isfinite(q[i]))return false;n+=q[i]*q[i];}
  if(fabsf(n-1)>.02f)return false;
  float length2=0;
  for(unsigned i=0;i<3;i++){if(!isfinite(r[i]))return false;length2+=r[i]*r[i];}
  if(length2>.090001f)return false;
  const float w=q[0],x=q[1],y=q[2],z=q[3];
  const float R[3][3]={{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)},
    {2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)},
    {2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}};
  const float minusSkew[3][3]={{0,r[2],-r[1]},{-r[2],0,r[0]},{r[1],-r[0],0}};
  for(unsigned i=0;i<3;i++) {
    offset[i]=-r[i];
    for(unsigned k=0;k<3;k++)offset[i]+=R[i][k]*r[k];
    for(unsigned j=0;j<3;j++) {
      J[i][j]=0;
      for(unsigned k=0;k<3;k++)J[i][j]+=R[i][k]*minusSkew[k][j];
    }
  }
  return true;
}

// Configured only while grounded; default-disabled on other missions/builds.
bool mocapMarkerSettings(float r[3]);
