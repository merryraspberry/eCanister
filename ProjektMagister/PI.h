struct PI {
    float kp;
    float ki;
    float min;
    float max;
    float Ts;
    float x;
    float y;
    float y1;
    float x1;
    float a;
    float b;
    float b1;
    float e;
    float flag;
};




void PI_Init(struct PI* pid , float kp, float ki, float min, float max, float Ts);
void PI_Calc(struct PI* pid , float x);

extern struct PI PI_i_DAB;
extern struct PI PI_u_DAB;
extern struct PI PI_i_PLL;
extern struct PI PI_u_PLL;
extern struct PI PI_PLL;
