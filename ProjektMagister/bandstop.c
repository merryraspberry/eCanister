#include "bandstop.h"
struct BAND BAND_1;



void band_Init(struct BAND *l, double  b, double  w, double  Ts) {
    l->b = b;
    l->w = w;
    l->Ts = Ts;
    l->x = 0;
    l->x1 = 0;
    l->x2 = 0;
    l->x3 = 0;
    l->x4 = 0;
    l->x5 = 0;
    l->y = 0;
    l->y1 = 0;
    l->y2 = 0;
    l->y3 = 0;
    l->y4 = 0;
    l->y5 = 0;
    l->num = 0;
    l->denum = 0;
    l->wt2 = 0;
    l->bt =0;
}

void band_Calc(struct BAND *l, double  x) {
    l->x = x;
    l->wt2 = l->w*l->w*l->Ts*l->Ts;
    l->bt = l->b*l->Ts;
    l->denum = ((l->wt2) +(2*l->bt) +4);

    l->y =
    ((l->wt2 + 4)*l->x)/(l->denum)
    +((5*l->wt2 + 4)*l->x1)/(l->denum)
    +((10*l->wt2 - 8)*l->x2)/(l->denum)
    +((10*l->wt2 - 8)*l->x3)/(l->denum)
    +((5*l->wt2 + 4)*l->x4)/(l->denum)
    +((l->wt2 + 4)*l->x5)/(l->denum)
    +((-5*l->wt2 - 6*l->bt - 4)*l->y1)/(l->denum)
    +((-10*l->wt2 - 4*l->bt + 8)*l->y2)/(l->denum)
    +((-10*l->wt2 + 4*l->bt + 8)*l->y3)/(l->denum)
    +((-5*l->wt2 + 6*l->bt - 4)*l->y4)/(l->denum)
    +((-1*l->wt2 + 2*l->bt - 4)*l->y5)/(l->denum);

    l->x5 = l->x4;
    l->x4 = l->x3;
    l->x3 = l->x2;
    l->x2 = l->x1;
    l->x1 = l->x;

    l->y5 = l->y4;
    l->y4 = l->y3;
    l->y3 = l->y2;
    l->y2 = l->y1;
    l->y1 = l->y;
}
