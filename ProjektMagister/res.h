struct R {
    float min;
    float max;
    float Ts;
    float x;
    float y;
    float y1;
    float x1;
    float b;
    float b1;

};




void R_Init(struct R* r ,float min, float max, float Ts);
void R_Calc(struct R* r , float x);

extern struct R R_1;

