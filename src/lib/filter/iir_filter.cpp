//
// Created by Sherry Wang on 2021/11/15.
//
#include "iir_filter.h"

IIRFilter::IIRFilter(double default_value)
{
    x_cur = x_delay_1 = x_delay_2 = 0.0;
    y_output = y_delay_1 = y_delay_2 = 0.0;
}
IIRFilter::~IIRFilter() {}

// double IIRFilter::sample(double value)
// {
//     x_delay_2 = x_delay_1; x_delay_1 = x_cur;
//     if (std::isnormal(value)) {
//         x_cur = value;
//     } else {
//         x_cur = 0.0;
//     }
//     y_output = 0.067455*x_cur + 0.134911*x_delay_1 + 0.067455*x_delay_2
//                + 1.14298*y_delay_1 - 0.412802*y_delay_2;
//     y_delay_2 = y_delay_1; y_delay_1 = y_output;
//     return y_output;
// }
double IIRFilter::sample(double value)
{
return value;
}

//double IIRFilter::getResult() {
//    /*
//		long double res = 0.022*rawdata_[5] + 0.11*rawdata_[4] + 0.219*rawdata_[3] + 0.219*rawdata_[2] + 0.11* rawdata_[1]+ 0.0219*rawdata_[0] - (-0.985)*filtered_[4] - 0.974*filtered_[3] - (-0.386)*filtered_[2] - 0.111*filtered_[1] - (-0.011)*filtered_[0];
//		std::cout << res << std::endl;
//    for (int i = 0; i < len_ - 1; i++) {
//        filtered_[i] = filtered_[i+1];
//    }
//    filtered_[4] = res;
//
//    return res;
//		*/
//		double zx = 0.067455*x0 + 0.134911*x1 + 0.067455*x2;
//		double zy = - 0.4128*y2; // 1.14298*y1 - 0.412802*y2;
//		std::cout << zx << " \t" << zy << std::endl;
//		y0 = zx + zy;
//		//std::cout << y0 << std::endl;
//		return y0;
//
//}
