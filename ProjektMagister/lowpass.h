struct LOW {
    double k;
    double d;
    double w;
    double Ts;
    double y;
    double y1;
    double y2;
    double y3;
    double x;
    double x1;
    double x2;
    double x3;
    double num;
    double denum;
    double wt2;
    double kwt2;
    double dwt;

};




void lowpass_Init(struct LOW* l , double k, double d, double w, double Ts);
void lowpass_Calc(struct LOW* l , double x);

extern struct LOW LOW_1;

