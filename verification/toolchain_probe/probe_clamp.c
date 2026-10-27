#include "j_core.h"
#include <stdio.h>
int main(void)
{
    volatile float integ=0,error=0,dt=0.003f;
    float before=integ+J_VZ_I*error*dt;
    float direct=J_Clamp(integ+J_VZ_I*error*dt,-J_VZ_I_LIMIT,J_VZ_I_LIMIT);
    float stored=J_Clamp(before,-J_VZ_I_LIMIT,J_VZ_I_LIMIT);
    printf("clamp runtime direct=%g stored=%g computed=%g limits=[%g,%g]\n",direct,stored,before,-J_VZ_I_LIMIT,J_VZ_I_LIMIT);
    return direct!=0.0f || stored!=0.0f;
}
