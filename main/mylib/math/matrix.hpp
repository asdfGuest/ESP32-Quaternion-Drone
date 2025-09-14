#ifndef MYLIB_MATRIX_HPP
#define MYLIB_MATRIX_HPP

#include <cstddef>
#include <stdexcept>


namespace mylib
{
    template<typename T>
    class Matrix
    {
    protected:
        T* _data;
        size_t _rows, _cols, _numel;

        bool is_index_valid(size_t row, size_t col, bool raise_exception = true) const
        {
            if (0 <= row && row < _rows && 0 <= col && col < _cols) {
                return true;
            }
            else {
                if (raise_exception) {
                    throw std::out_of_range("Matrix index out of range");
                }
                return false;
            }
        }
        static bool is_shape_same(const Matrix<T>& a, const Matrix<T>& b, bool raise_exception = true)
        {
            if (a._rows == b._rows && a._cols == b._cols) {
                return true;
            }
            else {
                if (raise_exception) {
                    throw std::invalid_argument("Matrix shapes are not the same");
                }
                return false;
            }
        }
        static bool is_shape_multipiable(const Matrix<T>& a, const Matrix<T>& b, bool raise_exception = true)
        {
            if (a._cols == b._rows) {
                return true;
            }
            else {
                if (raise_exception) {
                    throw std::invalid_argument("Matrix shapes are not multipliable");
                }
                return false;
            }
        }
    
    public:
        Matrix(const Matrix<T>& other)
        {
            _rows = other._rows;
            _cols = other._cols;
            _numel = other._numel;
            _data = new T[_numel];
            for (size_t i = 0; i < _numel; ++i) {
                _data[i] = other._data[i];
            }
        }
        Matrix(size_t rows, size_t cols)
        {
            _rows = rows;
            _cols = cols;
            _numel = rows * cols;
            _data = new T[_numel];
        }
        Matrix(size_t rows, size_t cols, T *init_arr)
        {
            _rows = rows;
            _cols = cols;
            _numel = rows * cols;
            _data = new T[_numel];
            for (size_t i = 0; i < _numel; ++i) {
                _data[i] = init_arr[i];
            }
        }
        Matrix(size_t rows, size_t cols, T init_val)
        {
            _rows = rows;
            _cols = cols;
            _numel = rows * cols;
            _data = new T[_numel];
            for (size_t i = 0; i < _numel; ++i) {
                _data[i] = init_val;
            }
        }
        ~Matrix()
        {
            delete[] _data;
        }
        
        size_t rows() const
        {
            return _rows;
        }
        size_t cols() const
        {
            return _cols;
        }
        size_t numel() const
        {
            return _numel;
        }
        T get(size_t row, size_t col) const
        {
            is_index_valid(row, col);
            return _data[row * _cols + col];
        }
        void set(size_t row, size_t col, T val)
        {
            is_index_valid(row, col);
            _data[row * _cols + col] = val;
        }


        static Matrix<T> add_inv(const Matrix<T>& a)
        {
            Matrix<T> result(a._rows, a._cols);
            for (size_t i = 0; i < a._numel; ++i) {
                result._data[i] = -a._data[i];
            }
            return result;
        }
        Matrix<T> add_inv(void) const
        {
            return add_inv(*this);
        }

        static Matrix<T> transpose(const Matrix<T>& a)
        {
            Matrix<T> result(a._cols, a._rows);
            for (size_t row = 0; row < a._rows; row++) {
                for (size_t col = 0; col < a._cols; col++) {
                    result.set(col, row, a.get(row, col));
                }
            }
            return result;
        }
        Matrix<T> transpose(void) const
        {
            return transpose(*this);
        }

        static Matrix<T> add(const Matrix<T>& a, const Matrix<T>& b)
        {
            is_shape_same(a, b);
            Matrix<T> result(a._rows, a._cols);
            for (size_t i = 0; i < a._numel; ++i) {
                result._data[i] = a._data[i] + b._data[i];
            }
            return result;
        }
        Matrix<T> add(const Matrix<T>& b) const
        {
            return add(*this, b);
        }

        static Matrix<T> mul(const Matrix<T>& a, const Matrix<T>& b)
        {
            is_shape_same(a, b);
            Matrix<T> result(a._rows, a._cols);
            for (size_t i = 0; i < a._numel; ++i) {
                result._data[i] = a._data[i] * b._data[i];
            }
            return result;
        }
        Matrix<T> mul(const Matrix<T>& b) const
        {
            return mul(*this, b);
        }

        static Matrix<T> mat_mul(const Matrix<T>& a, const Matrix<T>& b)
        {
            is_shape_multipiable(a, b);
            Matrix<T> result(a._rows, b._cols);
            for (size_t row = 0; row < a._rows; row++) {
                for (size_t col = 0; col < b._cols; col++) {
                    T sum = 0;
                    for (size_t k = 0; k < a._cols; k++) {
                        sum += a.get(row, k) * b.get(k, col);
                    }
                    result.set(row, col, sum);
                }
            }
            return result;
        }
        Matrix<T> mat_mul(const Matrix<T>& b) const
        {
            return mat_mul(*this, b);
        }

        Matrix<T>& operator=(const Matrix<T>& other)
        {
            if (this != &other) {
                delete[] _data;
                _rows = other._rows;
                _cols = other._cols;
                _numel = other._numel;
                _data = new T[_numel];
                for (size_t i = 0; i < _numel; ++i) {
                    _data[i] = other._data[i];
                }
            }
            return *this;
        }
    };
}

#endif
