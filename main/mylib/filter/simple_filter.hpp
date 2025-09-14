#ifndef MYLIB_SIMPLE_FILTER_HPP
#define MYLIB_SIMPLE_FILTER_HPP

#include "math.hpp"


namespace mylib {
    class SMAFilter {
    private:
        int buffer_size, buffer_ptr;
        double *buffer;
    public:
        SMAFilter(int buffer_size, double initial_value = 0.0);
        ~SMAFilter();
        double update(double new_value);
    };


    template<int ORDER>
    class LowPassFilter {
    private:
        double alpha, beta, buffer[ORDER];
    public:
        LowPassFilter(double alpha, double initial_value) {
            this->alpha = alpha;
            this->beta = 1.0 - alpha;
            for (int i = 0; i < ORDER; i++) {
                buffer[i] = initial_value;
            }
        }

        LowPassFilter(double sampling_freq, double cutoff_freq, double initial_value) {
            this->alpha = (mylib::TWO_PI * cutoff_freq) / (mylib::TWO_PI * cutoff_freq + sampling_freq);
            this->beta = 1.0 - alpha;
            for (int i = 0; i < ORDER; i++) {
                buffer[i] = initial_value;
            }
        }

        void clear(double initial_value) {
            for (int i = 0; i < ORDER; i++) {
                buffer[i] = initial_value;
            }
        }

        double update(double new_value) {
            buffer[0] = beta * buffer[0] + alpha * new_value;
            for (int i = 1; i < ORDER; i++) {
                buffer[i] = beta * buffer[i] + alpha * buffer[i - 1];
            }
            return buffer[ORDER - 1];
        }
    };


    template<int ORDER>
    class Vec3LowPassFilter {
    private:
        LowPassFilter<ORDER> x_lpf, y_lpf, z_lpf;
    public:
        Vec3LowPassFilter(double alpha, const mylib::Vector3<double>& initial_value)
            : x_lpf(alpha, initial_value.x),
              y_lpf(alpha, initial_value.y),
              z_lpf(alpha, initial_value.z) {}

        Vec3LowPassFilter(double sampling_freq, double cutoff_freq, const mylib::Vector3<double>& initial_value)
            : x_lpf(sampling_freq, cutoff_freq, initial_value.x),
              y_lpf(sampling_freq, cutoff_freq, initial_value.y),
              z_lpf(sampling_freq, cutoff_freq, initial_value.z) {}

        void clear(const mylib::Vector3<double>& initial_value) {
            x_lpf.clear(initial_value.x);
            y_lpf.clear(initial_value.y);
            z_lpf.clear(initial_value.z);
        }

        mylib::Vector3<double> update(const mylib::Vector3<double>& new_value) {
            return mylib::Vector3<double>(
                x_lpf.update(new_value.x),
                y_lpf.update(new_value.y),
                z_lpf.update(new_value.z)
            );
        }
    };
}

#endif
