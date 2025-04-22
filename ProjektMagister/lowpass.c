#include "lowpass.h"
struct LOW LOW_1;


void lowpass_Init(struct LOW *l, double k, double d, double w, double Ts) {
    l->k = k;
    l->d = d;
    l->w = w;
    l->Ts = Ts;
    l->x = 0;
    l->x1 = 0;
    l->x2 = 0;
    l->x3 = 0;
    l->y = 0;
    l->y1 = 0;
    l->y2 = 0;
    l->y3 = 0;
    l->num = 0;
    l->denum = 0;
    l->wt2 = 0;
    l->kwt2= 0;
    l->dwt =0;


}

void lowpass_Calc(struct LOW *l, double x) {
    l->x = x;
    l->wt2 = l->w*l->w*l->Ts*l->Ts;
    l->kwt2 = l->wt2*l->k;
    l->dwt = l->d*l->w*l->Ts;

    l->num =
    ((l->kwt2*l->x)
    +(3*l->kwt2*l->x1)
    +(3*l->kwt2*l->x2)
    +(l->kwt2*l->x3)

    -(3*l->wt2*l->y1)
    -(4*l->dwt*l->y1)

    -(3*l->wt2*l->y2)
    +(4*l->dwt*l->y2)

    -(l->wt2*l->y3)
    +(4*l->dwt*l->y3)

    +(4*l->y1)
    +(4*l->y2)
    -(4*l->y3));

    l->denum = ((l->wt2)  +  (4*l->dwt)  +  4);

    l->y = (l->num) / (l->denum);


    l->x3 = l->x2;
    l->x2 = l->x1;
    l->x1 = l->x;



    l->y3 = l->y2;
    l->y2 = l->y1;
    l->y1 = l->y;


}
