#include "kinova_test/Matrix.h"

// ---------- Constructor Start ----------
template <class DATA_TYPE>
Matrix<DATA_TYPE>::Matrix(unsigned rows, unsigned cols) : _rows(rows), _cols(cols)
{
    if (rows == 0 || cols == 0)
        throw std::logic_error("The rows and cols can not be zero.");
    matrix = new DATA_TYPE[_rows * _cols]();
}

template <class DATA_TYPE>
Matrix<DATA_TYPE>::Matrix(unsigned rows, unsigned cols, MatrixType type, std::initializer_list<DATA_TYPE> l) : _rows(rows), _cols(cols)
{
    if (rows == 0 || cols == 0)
        throw std::logic_error("The rows and cols can not be zero.");
    matrix = new DATA_TYPE[_rows * _cols]();
    unsigned k = 0;
    if (type == MatrixType::Diagonal)
    {
        if (rows != cols)
            throw std::logic_error("Must be square matrix.");
        if (l.size() != rows)
            throw std::logic_error("The numbers of list must equal to matrix dimension.");
        for (auto i = l.begin(); i != l.end(); i++, k++)
            *(matrix + _cols * k + k) = *i;
    }
    else if (type == MatrixType::General)
    {
        if (l.size() > rows * cols)
            throw std::logic_error("The numbers of list must less or equal to matrix dimension.");
        for (auto i = l.begin(); i != l.end(); i++, k++)
            *(matrix + k) = *i;
    }
}

template <class DATA_TYPE>
Matrix<DATA_TYPE>::Matrix(const Matrix<DATA_TYPE> &mat) : _rows(mat._rows), _cols(mat._cols)
{
    matrix = new DATA_TYPE[_rows * _cols];
    std::copy(mat.matrix, mat.matrix + _rows * _cols, matrix);
}
// ---------- Constructor End ----------

// ---------- Destructor Start ----------
template <class DATA_TYPE>
Matrix<DATA_TYPE>::~Matrix()
{
    delete[] matrix;
    matrix = nullptr;
}
// ---------- Destructor End ----------

// ---------- Operator Start ----------
template <class DATA_TYPE>
Matrix<DATA_TYPE> &Matrix<DATA_TYPE>::operator=(const Matrix<DATA_TYPE> &mat)
{
    if (this == &mat)
        return *this;
    if (_rows != mat._rows || _cols != mat._cols)
    {
        delete[] matrix;
        _rows = mat._rows;
        _cols = mat._cols;
        matrix = new DATA_TYPE[_rows * _cols]();
    }
    std::copy(mat.matrix, mat.matrix + _rows * _cols, matrix);
    return *this;
}

template <class DATA_TYPE>
const DATA_TYPE &Matrix<DATA_TYPE>::operator[](unsigned num) const
{
    if (num >= _rows * _cols)
        throw std::out_of_range("Invalid index to matrix");
    return *(matrix + num);
}

template <class DATA_TYPE>
DATA_TYPE &Matrix<DATA_TYPE>::operator[](unsigned num)
{
    if (num >= _rows * _cols)
        throw std::out_of_range("Invalid index to matrix");
    return *(matrix + num);
}

template <class DATA_TYPE>
const DATA_TYPE &Matrix<DATA_TYPE>::operator()(unsigned row, unsigned col) const
{
    if (row >= _rows || col >= _cols)
        throw std::out_of_range("Invalid index to matrix");
    return *(matrix + _cols * row + col);
}

template <class DATA_TYPE>
DATA_TYPE &Matrix<DATA_TYPE>::operator()(unsigned row, unsigned col)
{
    if (row >= _rows || col >= _cols)
        throw std::out_of_range("Invalid index to matrix");
    return *(matrix + _cols * row + col);
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator+(const Matrix<DATA_TYPE> &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    Matrix<DATA_TYPE> ret(_rows, _cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) + *(rhs.matrix + _cols * i + j);
    return ret;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> &Matrix<DATA_TYPE>::operator+=(const Matrix<DATA_TYPE> &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(matrix + _cols * i + j) += *(rhs.matrix + _cols * i + j);
    return *this;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator-(const Matrix<DATA_TYPE> &rhs)
{
    Matrix<DATA_TYPE> ret(_rows, _cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) - *(rhs.matrix + _cols * i + j);
    return ret;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> &Matrix<DATA_TYPE>::operator-=(const Matrix<DATA_TYPE> &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(matrix + _cols * i + j) -= *(rhs.matrix + _cols * i + j);
    return *this;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator*(const DATA_TYPE rhs)
{
    Matrix<DATA_TYPE> ret(_rows, _cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) * rhs;
    return ret;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> &Matrix<DATA_TYPE>::operator*=(const DATA_TYPE rhs)
{
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(matrix + _cols * i + j) *= rhs;
    return *this;
}
template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator*(const DATA_TYPE rhs) const
{
    Matrix<DATA_TYPE> ret(_rows, _cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) * rhs;
    return ret;
}
template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator*(const Matrix<DATA_TYPE> &rhs)
{
    if (_cols != rhs._rows)
        throw std::logic_error("LHS column is not equal to RHS row.");
    Matrix<DATA_TYPE> ret(_rows, rhs._cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < rhs._cols; j++)
            for (unsigned k = 0; k < _cols; k++)
                *(ret.matrix + rhs._cols * i + j) += *(matrix + _cols * i + k) * *(rhs.matrix + rhs._cols * k + j);
    return ret;
}
template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator*(const Matrix<DATA_TYPE> &rhs) const
{
    if (_cols != rhs._rows)
        throw std::logic_error("LHS column is not equal to RHS row.");
    Matrix<DATA_TYPE> ret(_rows, rhs._cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < rhs._cols; j++)
            for (unsigned k = 0; k < _cols; k++)
                *(ret.matrix + rhs._cols * i + j) += *(matrix + _cols * i + k) * *(rhs.matrix + rhs._cols * k + j);
    return ret;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> &Matrix<DATA_TYPE>::operator*=(const Matrix<DATA_TYPE> &rhs)
{
    if (_cols != rhs._rows)
        throw std::logic_error("LHS column is not equal to RHS row.");
    Matrix<DATA_TYPE> ret(_rows, rhs._cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < rhs._cols; j++)
            for (unsigned k = 0; k < _cols; k++)
                *(ret.matrix + rhs._cols * i + j) += *(matrix + _cols * i + k) * *(rhs.matrix + rhs._cols * k + j);
    return *this = ret;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::operator/(const DATA_TYPE rhs)
{
    Matrix<DATA_TYPE> ret(_rows, _cols);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) / rhs;
    return ret;
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> &Matrix<DATA_TYPE>::operator/=(const DATA_TYPE rhs)
{
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(matrix + _cols * i + j) /= rhs;
    return *this;
}
// ---------- Operator End ----------

// ---------- Other function Start ----------
template <class DATA_TYPE>
void Matrix<DATA_TYPE>::update_from_matlab(const DATA_TYPE *arr)
{
    for (unsigned i = 0, k = 0; i < _cols; i++)
        for (unsigned j = 0; j < _rows; j++, k++)
            *(matrix + _cols * j + i) = *(arr + k);
}

template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::transpose()
{
    Matrix<DATA_TYPE> T(_cols, _rows);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(T.matrix + _rows * j + i) = *(matrix + _cols * i + j);
    return T;
}
template <class DATA_TYPE>
Matrix<DATA_TYPE> Matrix<DATA_TYPE>::transpose() const
{
    Matrix<DATA_TYPE> T(_cols, _rows);
    for (unsigned i = 0; i < _rows; i++)
        for (unsigned j = 0; j < _cols; j++)
            *(T.matrix + _rows * j + i) = *(matrix + _cols * i + j);
    return T;
}

template <class DATA_TYPE>
void Matrix<DATA_TYPE>::zeros()
{
    for (unsigned i = 0; i < _rows * _cols; i++)
        *(matrix + i) = 0;
}

template <class DATA_TYPE>
Matrix<double> Matrix<DATA_TYPE>::inverse()
{
    if (_rows != _cols)
        throw std::logic_error("Must be square matrix.");
    Matrix<double> adjugate(_rows, _cols);
    if (_rows != 1)
    {
        Matrix<DATA_TYPE> minorMat(_rows - 1, _cols - 1);
        for (unsigned r = 0; r < _rows; r++)
        {
            for (unsigned c = 0; c < _rows; c++)
            {
                for (unsigned i = 0, mat_i = 0; i < _rows; i++)
                {
                    if (i == r)
                        continue;
                    for (unsigned j = 0, mat_j = 0; j < _rows; j++)
                    {
                        if (j == c)
                            continue;
                        minorMat(mat_i, mat_j++) = *(matrix + _cols * i + j);
                    }
                    mat_i++;
                }
                if ((r + c) % 2 == 0)
                    adjugate(c, r) = det(minorMat);
                else
                    adjugate(c, r) = -det(minorMat);
            }
        }
    }
    else
        adjugate(0, 0) = *(matrix) ? 1 : 0;
    return adjugate / det(*this);
}

// ---------- Other function End ----------

template class Matrix<double>;
template class Matrix<float>;
template class Matrix<int>;
