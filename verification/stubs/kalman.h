#ifndef J_MOCK_KALMAN
#define J_MOCK_KALMAN
struct _1_ekf_filter { float a,b,out,c,d,e; };
void kalman_1(struct _1_ekf_filter *filter,float value);
#endif
