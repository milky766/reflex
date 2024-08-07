//
// Created by Sherry Wang on 2021/7/3.
//

#ifndef CONTROL_BOARD_RT_AVE_H
#define CONTROL_BOARD_RT_AVE_H


class RTLoopHistory {
public:
    /**
     * Constructor
     * @param length amount of historical samples to use in calculation of arithmetic mean
     * @param default_value assumed loop rate (hz) for initializing the history
     */
    RTLoopHistory(unsigned length, double default_value);

    /**
     * Destructor
     */
    ~RTLoopHistory();

    /**
     * Adds a new sample to the history
     * @param value loop rate
     */
    void sample(double value);

    /**
     * Calculates arithmetic mean of historic loop rates
     * @return arithmetic mean of last [length] loop rates
     */
    double average() const;

protected:
    unsigned index_;
    unsigned length_;
    double *history_;
};

#endif //CONTROL_BOARD_RT_AVE_H
