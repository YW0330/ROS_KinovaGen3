#include "kinova_test/Matrix.h"

// ---------- Constructor Start ----------
Matrix::Matrix(int rows, int cols) : _rows(rows), _cols(cols)
{
    matrix = new double *[_rows];
    for (int i = 0; i < _rows; i++)
        matrix[i] = new double[_cols];
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] = 0;
}

Matrix::Matrix(const Matrix &mat) : _rows(mat._rows), _cols(mat._cols)
{
    matrix = new double *[_rows];
    for (int i = 0; i < _rows; i++)
        matrix[i] = new double[_cols];
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] = mat.matrix[i][j];
}
// ---------- Constructor End ----------

// ---------- Destructor Start ----------
Matrix::~Matrix()
{
    if (matrix == nullptr)
        return;
    for (int i = 0; i < _rows; i++)
        delete[] matrix[i];
    delete[] matrix;
    matrix = nullptr;
}
// ---------- Destructor End ----------

// ---------- Operator Start ----------
Matrix &Matrix::operator=(const Matrix &mat)
{
    if (this == &mat)
        return *this;
    if (_rows != mat._rows || _cols != mat._cols)
    {
        this->~Matrix();
        _rows = mat._rows;
        _cols = mat._cols;
        matrix = new double *[_rows];
        for (int i = 0; i < _rows; i++)
            matrix[i] = new double[_cols];
    }
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] = mat.matrix[i][j];
    return *this;
}

const double &Matrix::operator[](int num) const
{
    if (num < 0 || (num >= _rows && num >= _cols))
        throw std::out_of_range("Invalid index to matrix");
    return _rows == 1 ? matrix[0][num] : matrix[num][0];
}

double &Matrix::operator[](int num)
{
    if (num < 0 || (num >= _rows && num >= _cols))
        throw std::out_of_range("Invalid index to matrix");
    return _rows == 1 ? matrix[0][num] : matrix[num][0];
}

const double &Matrix::operator()(int row, int col) const
{
    if (row < 0 || col < 0 || row >= _rows || col >= _cols)
        throw std::out_of_range("Invalid index to matrix");
    return matrix[row][col];
}

double &Matrix::operator()(int row, int col)
{
    if (row < 0 || col < 0 || row >= _rows || col >= _cols)
        throw std::out_of_range("Invalid index to matrix");
    return matrix[row][col];
}

Matrix Matrix::operator+(const Matrix &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    Matrix ret(_rows, _cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            ret.matrix[i][j] = matrix[i][j] + rhs.matrix[i][j];
    return ret;
}

Matrix &Matrix::operator+=(const Matrix &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] += rhs.matrix[i][j];
    return *this;
}

Matrix Matrix::operator-(const Matrix &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    Matrix ret(_rows, _cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            ret.matrix[i][j] = matrix[i][j] - rhs.matrix[i][j];
    return ret;
}

Matrix &Matrix::operator-=(const Matrix &rhs)
{
    if (_rows != rhs._rows || _cols != rhs._cols)
        throw std::logic_error("LHS size is not equal to RHS size.");
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] -= rhs.matrix[i][j];
    return *this;
}

Matrix Matrix::operator*(const double rhs)
{
    Matrix ret(_rows, _cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            ret.matrix[i][j] = matrix[i][j] * rhs;
    return ret;
}

Matrix Matrix::operator*(const Matrix &rhs)
{
    if (_cols != rhs._rows)
        throw std::logic_error("LHS column is not equal to RHS row.");
    Matrix ret(_rows, rhs._cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < rhs._cols; j++)
            for (int k = 0; k < _cols; k++)
                ret.matrix[i][j] += matrix[i][k] * rhs.matrix[k][j];
    return ret;
}

Matrix &Matrix::operator*=(const double rhs)
{
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] *= rhs;
    return *this;
}

Matrix &Matrix::operator*=(const Matrix &rhs)
{
    if (_cols != rhs._rows)
        throw std::logic_error("LHS column is not equal to RHS row.");
    Matrix ret(_rows, rhs._cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < rhs._cols; j++)
            for (int k = 0; k < _cols; k++)
                ret.matrix[i][j] += matrix[i][k] * rhs.matrix[k][j];
    return *this = ret;
}

Matrix Matrix::operator/(const double rhs)
{
    Matrix ret(_rows, _cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            ret.matrix[i][j] = matrix[i][j] / rhs;
    return ret;
}

Matrix &Matrix::operator/=(const double rhs)
{
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] /= rhs;
    return *this;
}

std::ostream &operator<<(std::ostream &os, const Matrix &mat)
{
    for (int i = 0; i < mat._rows; i++)
    {
        for (int j = 0; j < mat._cols; j++)
            std::cout << mat.matrix[i][j] << "\t";
        std::cout << std::endl;
    }
    return os;
}
// ---------- Operator End ----------

// ---------- Other function Start ----------
void Matrix::update_from_matlab(double *arr)
{
    for (int i = 0, k = 0; i < _cols; i++)
        for (int j = 0; j < _rows; j++, k++)
            matrix[j][i] = arr[k];
}

Matrix Matrix::gen_Transpose()
{
    Matrix T(_cols, _rows);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            T.matrix[j][i] = matrix[i][j];
    return T;
}
// ---------- Other function End ----------
