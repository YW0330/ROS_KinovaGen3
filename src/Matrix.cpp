#include "kinova_test/Matrix.h"

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
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] = mat.matrix[i][j];
}

Matrix::~Matrix()
{
    for (int i = 0; i < _rows; i++)
        delete[] matrix[i];
    delete[] matrix;
    matrix = nullptr;
}

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
    std::copy(mat.matrix, mat.matrix + _rows * _cols, matrix);
    return *this;
}

const double &Matrix::operator()(int row, int col) const
{
    // 索引錯誤
    if (row < 0 || col < 0 || row > _rows || col > _cols)
        throw std::out_of_range("Invalid index to matrix");
    return matrix[row][col];
}

double &Matrix::operator()(int row, int col)
{
    // 索引錯誤
    if (row < 0 || col < 0 || row > _rows || col > _cols)
        throw std::out_of_range("Invalid index to matrix");
    return matrix[row][col];
}

Matrix &Matrix::operator*(const double &rhs)
{
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] *= rhs;
    return *this;
}

Matrix &operator*(const double &lhs, Matrix &mat)
{
    return mat * lhs;
}

Matrix &Matrix::operator/(const double &rhs)
{
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < _cols; j++)
            matrix[i][j] /= rhs;
    return *this;
}

Matrix &operator/(const double &lhs, Matrix &mat)
{
    return mat / lhs;
}

Matrix Matrix::operator*(const Matrix &rhs)
{
    if (_cols != rhs._rows)
        throw std::out_of_range("LHS columns is not equal to RHS rows.");
    Matrix ret(_rows, rhs._cols);
    for (int i = 0; i < _rows; i++)
        for (int j = 0; j < rhs._cols; j++)
            for (int k = 0; k < _cols; k++)
                ret.matrix[i][j] += matrix[i][k] * rhs.matrix[k][j];
    return ret;
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
