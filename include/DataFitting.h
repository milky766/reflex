#ifndef DATA_FITTING_H
#define DATA_FITTING_H

#include <iostream>
#include <math.h>

class DataFitting //此class被用来对tension senosr进行最小二乘法拟合来实现信号校准
{
public:

    struct Slope_Intercept
    {
        double Slope;//斜率
        double Intercept;//截距
        Slope_Intercept(): Slope(0), Intercept(0) { }//结构体的重载型constructor，这里使用的是initializer的形式
    };
    
    


    double getAverage(double Rdata[], int count);//求数据平均值,Rdata = Raw Data
    Slope_Intercept getSlope_Intercept(double Rdata_a[], double Rdata_anta[], int count);//求斜率和截距
    //注意现有tension sensor的信号是uint16_t的数据类型，在传入此函数时需要进行向double的转换
    //实在不行把double改成uint16_t就好了，我只是单纯很菜不想碰uint16_t
private:

    Slope_Intercept Slope_Intercept_;//类内部使用的结构体，call的时候需要新实例化一个结构体来接受结果
    double Rdata_a_,Rdata_anta_;
    double AverageV_a;
    double AverageV_anta;
    int count_a,count_anta;
    double upper;
    double lower;
};

#endif