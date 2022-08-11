#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
class Matrix
{
private:
    double **matrix;
    int _rows;
    int _cols;

public:
    Matrix(int rows, int cols);
    Matrix(const Matrix &mat);
    ~Matrix();
    Matrix &operator=(const Matrix &mat);
    const double &operator()(int row, int col) const;
    double &operator()(int row, int col);
    Matrix &operator*(const double &rhs);
    friend Matrix &operator*(const double &lhs, Matrix &mat);
    Matrix &operator/(const double &rhs);
    friend Matrix &operator/(const double &lhs, Matrix &mat);
    Matrix operator*(const Matrix &rhs);
    void update_from_matlab(double *arr);
    Matrix gen_Transpose();
    friend std::ostream &operator<<(std::ostream &os, const Matrix &mat);
};

#endif