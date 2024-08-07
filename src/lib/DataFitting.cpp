#include "DataFitting.h"


double DataFitting::getAverage(double Rdata[], int count)
{
    double sum = 0;

    for (int i =0; i < count; i++)
    {
        sum += Rdata[i];
    }
    
    double AverageV = sum/count;
    return AverageV;
}



DataFitting::Slope_Intercept DataFitting::getSlope_Intercept(double Rdata_a[], double Rdata_anta[], int count)
{
    
    AverageV_a = getAverage(Rdata_a, count);
    AverageV_anta =  getAverage(Rdata_anta, count);
    count_a = count;
    count_anta = count;
    upper = 0;
    lower = 0;

    for(int i = 0; i < count_a; i++)
    {
        upper += ((Rdata_a[i] - AverageV_a) * (Rdata_anta[i] - AverageV_anta));
        lower += pow((Rdata_a[i] - AverageV_a),(double) 2);
    }

    Slope_Intercept_.Slope = upper/lower;
    Slope_Intercept_.Intercept = AverageV_anta - Slope_Intercept_.Slope * AverageV_a;
    return Slope_Intercept_;
}


