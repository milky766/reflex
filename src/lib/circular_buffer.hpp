//
// Created by Sherry Wang on 2021/5/17.
//

#ifndef CONTROL_BOARD_CIRCULAR_BUFFER_HPP
#define CONTROL_BOARD_CIRCULAR_BUFFER_HPP

#include <boost/circular_buffer.hpp>

template <class T>
class CircularBuffer{
public:
    CircularBuffer();
     ~CircularBuffer();
    void addToBuffer(T value);
    T getAverage();

private:
    int len_;
    T sum_ = 0;
    boost::circular_buffer<T> c_buffer_ = boost::circular_buffer<T>(500);
};

template <class T>
CircularBuffer<T>::CircularBuffer()
{
    //this->len_ = len;
    //this->c_buffer_.set_capacity(len_);
}

template <class T>
CircularBuffer<T>::~CircularBuffer() {
}

template <class T>
void CircularBuffer<T>::addToBuffer(T value)
{
    if (!c_buffer_.full()) {
        sum_ += value;
    } else {
        sum_ += (value - c_buffer_.front());
    }
    c_buffer_.push_back(value);
}

template <class T>
T CircularBuffer<T>::getAverage() {
    if (c_buffer_.empty()){
        return 0;
    }
    else {
        return sum_/c_buffer_.size();
    }
}

#endif //CONTROL_BOARD_CIRCULAR_BUFFER_HPP
