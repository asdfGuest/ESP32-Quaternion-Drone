#include "utility.hpp"


double mylib::apply_cubic_rc_rate(double input, double rate)
{
    // ax^3 + (1-a)x
    return (rate * input * input + (1.0 - rate)) * input;
}


mylib::DiscreteAuxMapper::DiscreteAuxMapper(int aux_min, int aux_max, int discrete_num, bool inverse)
{
    this->aux_min = aux_min;
    this->aux_max = aux_max;
    this->discrete_num = discrete_num;
    this->inverse = inverse;
}

int mylib::DiscreteAuxMapper::map(int aux_val) const
{
    if (inverse) {
        aux_val = aux_max + aux_min - aux_val;
    }
    aux_val = std::clamp(aux_val, aux_min, aux_max);

    // inequality: (aux_max - aux_min + 1) * idx <= (aux_val - aux_min) * discrete_num < (aux_max - aux_min + 1) * (idx + 1)
    // -> idx = floor((aux_val - aux_min) * discrete_num / (aux_max - aux_min + 1))
    return (aux_val - aux_min) * discrete_num / (aux_max - aux_min + 1);
}


mylib::ContinuousAuxMapper::ContinuousAuxMapper(int aux_min, int aux_max, bool inverse)
{
    this->aux_min = aux_min;
    this->aux_max = aux_max;
    this->inverse = inverse;
}

double mylib::ContinuousAuxMapper::map(int aux_val) const
{
    if (inverse) {
        aux_val = aux_max + aux_min - aux_val;
    }
    aux_val = std::clamp(aux_val, aux_min, aux_max);

    return (double)(aux_val - aux_min) / (double)(aux_max - aux_min);
}
