/* Offline ctypes adapter. All estimation is the frozen firmware C kernel. */
#include <math.h>
#include <string.h>
#include "post_release_inertial_ekf.h"
#include "post_release_vicon_velocity_kf.h"
static postReleaseInertialEkf_t filter;
static postReleaseInertialEkfParams_t params;
static postReleaseViconVelocityKf_t frontend;
int native_init(const float *p,const float *v,const float *q,
                const float *bg,const float *ba,unsigned t) {
  postReleaseInertialEkfDefaultParams(&params);
  // Replay consumes 100 Hz log snapshots, not the high-rate onboard stream.
  params.maxImuGapUs=20000;
  postReleaseViconVelocityKfReset(&frontend);
  Axis3f gyro={.axis={bg[0],bg[1],bg[2]}};
  int ok=postReleaseInertialEkfInit(&filter,&params,p,v,q,bg,&gyro,t);
  if(ok)memcpy(filter.accelBiasMps2,ba,sizeof(float)*3);
  return ok;
}
int native_propagate(const float *gyro,const float *acc,unsigned t) {
  Axis3f g,a;memcpy(g.axis,gyro,sizeof(float)*3);memcpy(a.axis,acc,sizeof(float)*3);
  postReleaseInertialEkfPropagate(&filter,&params,&g,&a,t);return filter.valid;
}
void native_previous_gyro(const float *gyro) {
  memcpy(filter.previousGyroRadPerS.axis,gyro,3*sizeof(float));
  filter.previousGyroValid=true;
}
int native_raw(const float *p,float std,unsigned t) {
  return postReleaseInertialEkfUpdatePosition(&filter,&params,p,std,t);
}
int native_frontend(const float *p,unsigned t,float prior) {
  if(!postReleaseViconVelocityKfUpdate(&frontend,p,t)||!frontend.valid)return 0;
  float position[3],R[3][2][2];
  const float dt=(filter.lastTimestampUs-t)*1.e-6f;
  for(int k=0;k<3;k++) {
    position[k]=frontend.positionM[k]+dt*frontend.velocityMps[k];
    R[k][0][0]=frontend.covariance[k][0][0]+2*dt*frontend.covariance[k][0][1]+
      dt*dt*frontend.covariance[k][1][1]+1.e-6f;
    R[k][0][1]=R[k][1][0]=frontend.covariance[k][0][1]+dt*frontend.covariance[k][1][1];
    R[k][1][1]=frontend.covariance[k][1][1]+1.e-4f;
  }
  return postReleaseInertialEkfFuseVicon(&filter,position,frontend.velocityMps,R,prior,t);
}
int native_pv(const float *p,const float *v,unsigned t,float posvar,float velvar) {
  float R[3][2][2]={0};
  for(int k=0;k<3;k++){R[k][0][0]=posvar;R[k][1][1]=velvar;}
  return postReleaseInertialEkfFuseVicon(&filter,p,v,R,.999f,t);
}
#ifdef WITH_DELAYED_UPDATE
int native_delayed(const float *p,float std,unsigned capture,float uncertainty) {
  return postReleaseInertialEkfUpdatePositionDelayed(&filter,&params,p,std,capture,uncertainty,2.f);
}
#endif
void native_state(float *output) {
  memcpy(output,filter.positionM,3*sizeof(float));
  memcpy(output+3,filter.velocityMps,3*sizeof(float));
  memcpy(output+6,filter.quaternionWxyz,4*sizeof(float));
  memcpy(output+10,filter.gyroBiasRadPerS,3*sizeof(float));
  memcpy(output+13,filter.accelBiasMps2,3*sizeof(float));
  memcpy(output+16,frontend.velocityMps,3*sizeof(float));
  output[19]=filter.valid;output[20]=filter.viconFusionCount;
  output[21]=filter.rejectedViconFusionCount;output[22]=filter.positionUpdateCount;
  output[23]=filter.rejectedPositionCount;
}
