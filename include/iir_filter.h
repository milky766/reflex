//
// Created by Sherry Wang on 2021/11/15.
//

#ifndef CONTROL_BOARD_IIR_FILTER_H
#define CONTROL_BOARD_IIR_FILTER_H

//A second order filter.
//parameters are already defined.
#include <iostream>
#include <cmath>
class IIRFilter
{
public:
    IIRFilter(double default_value);
    ~IIRFilter();
    double sample(double value);

private:
    double x_cur, x_delay_1, x_delay_2;
    double y_output, y_delay_1, y_delay_2;

};


#endif //CONTROL_BOARD_IIR_FILTER_H
