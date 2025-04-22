#include "res.h"
struct R R_1;

void R_Init(struct R *r, float min, float max, float Ts) {

    r->min = min;
    r->max = max;
    r->Ts = Ts;
    r->x = 0;
    r->x1 = 0;
    r->y = 0;
    r->y1 = 0;
    r->b = 0;
    r->b1 = 0;

}

void R_Calc(struct R *r, float x) {
    r->x = x;

    r->b = r->b1 + ((r->x + r->x1) * r->Ts) / 2;

    //pierwsze ograniczenie
    if (r->b > r->max) {
        r->b = 0;
    }
    if (r->b < r->min) {
        r->b = 0;
    }

    r->y = r->b;
    r->b1 = r->b;
    r->x1 = r->x;
    r->y1 = r->y;
}
