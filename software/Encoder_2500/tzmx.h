#ifndef TZ_H
#define TZ_H


#include "main.h"

#define L_Udc 5.0f

void ref_deal_core(void);
void tzmx_c_Outputs_wrapper(const float *iqk,
                            const float *n_error,
                            const float *nk,
                            float *uqk);



#endif
