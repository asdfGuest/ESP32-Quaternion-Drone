#include "simple_filter.hpp"


mylib::SMAFilter::SMAFilter(int buffer_size, double initial_value) {
    this->buffer_size = buffer_size;
    this->buffer_ptr = 0;
    this->buffer = new double[buffer_size]();

    for (int i = 0; i < buffer_size; i++) {
        buffer[i] = initial_value;
    }
}
mylib::SMAFilter::~SMAFilter() {
    delete[] buffer;
}
double mylib::SMAFilter::update(double new_value) {
    buffer[buffer_ptr] = new_value;
    buffer_ptr = (buffer_ptr + 1) % buffer_size;

    double sum = 0.0;
    for (int i = 0; i < buffer_size; i++) {
        sum += buffer[i];
    }
    return sum / buffer_size;
}
