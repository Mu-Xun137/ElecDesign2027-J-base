#include "pid.h"
static float clip(float x,float lo,float hi) { return x<lo?lo:(x>hi?hi:x); }
void pidRest(PidObject **p,const uint8_t len)
{
    uint8_t k;
    for(k=0;k<len;++k) {
        p[k]->measured=p[k]->desired=p[k]->integ=p[k]->prevError=p[k]->out=0.0f;
        p[k]->prevMeasured=0.0f; p[k]->initialized=0;
    }
}
void pidUpdate(PidObject *p,const float dt)
{
    float error,derivative,integral,raw;
    if(dt<=0.0f || dt>0.1f) return;
    error=p->desired-p->measured+p->offset;
    derivative=p->initialized?-(p->measured-p->prevMeasured)/dt:0.0f;
    integral=p->integ+error*dt;
    integral=clip(integral,p->IntegLimitLow,p->IntegLimitHigh);
    raw=p->kp*error+p->ki*integral+p->kd*derivative;
    if(!((raw>p->OutLimitHigh && error>0.0f)||(raw<p->OutLimitLow && error<0.0f))) p->integ=integral;
    raw=p->kp*error+p->ki*p->integ+p->kd*derivative;
    p->out=clip(raw,p->OutLimitLow,p->OutLimitHigh);
    p->prevError=error; p->prevMeasured=p->measured; p->initialized=1;
}
void CascadePID(PidObject *rate,PidObject *angle,const float dt)
{ pidUpdate(angle,dt); rate->desired=angle->out; pidUpdate(rate,dt); }
