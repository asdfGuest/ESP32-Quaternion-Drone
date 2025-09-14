#ifndef MYLIB_UTILITY_HPP
#define MYLIB_UTILITY_HPP

#include <algorithm>

namespace mylib
{
    double apply_cubic_rc_rate(double input, double rate);

    class DiscreteAuxMapper {
    private:
        int aux_min;
        int aux_max;
        int discrete_num;
        bool inverse;
    public:
        DiscreteAuxMapper(int aux_min, int aux_max, int discrete_num, bool inverse = false);
        int map(int aux_val) const;
    };

    class ContinuousAuxMapper {
    private:
        int aux_min;
        int aux_max;
        bool inverse;
    public:
        ContinuousAuxMapper(int aux_min, int aux_max, bool inverse = false);
        double map(int aux_val) const;
    };
}


#endif
