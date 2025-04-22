struct BAND {
    double  b;
    double  w;
    double  Ts;
    double  y;
    double  y1;
    double  y2;
    double  y3;
    double  y4;
    double  y5;
    double  x;
    double  x1;
    double  x2;
    double  x3;
    double  x4;
    double  x5;
    double  num;
    double  denum;
    double  wt2;
    double  bt;

};




void bandstop_Init(struct BAND* l , double b, double w, double Ts);
void bandstop_Calc(struct BAND* l , double x);

extern struct BAND BAND_1;
