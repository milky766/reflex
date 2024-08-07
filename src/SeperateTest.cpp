#include"DataFitting.h"


int main()
{

double X[10] = {0,1,5,8,6,11,10,8,15,19};
double Y[10] = {0,1,2,3,4,5,6,7,8,9};


DataFitting TensionMapping_;
DataFitting::Slope_Intercept result_;

result_ = TensionMapping_.getSlope_Intercept(X, Y);

std::cout << "Slope = " << result_.Slope << std::endl;
std::cout << "Intercept = " << result_.Intercept << std::endl;

}