#ifndef MATRIX_H_
#define MATRIX_H_

#include <iostream>
class Matrix
{
private:
    double **matrix;
    int _rows;
    int _cols;

public:
    /*
     * 產生一個矩陣，初始值為0。
     *
     * @param rows: 矩陣列數， cols: 矩陣行數
     *
     */
    Matrix(int rows, int cols);
    Matrix(const Matrix &mat);
    ~Matrix();
    Matrix &operator=(const Matrix &mat);

    /*
     * 向量取值。
     *
     * @param 元素編號
     *
     */
    const double &operator[](int num) const;
    double &operator[](int num);
    /*
     * 矩陣取值。
     *
     * @param row: 元素所在列， col: 元素所在行
     *
     */
    const double &operator()(int row, int col) const;
    double &operator()(int row, int col);

    /*
     * 生成一個由原矩陣各元素加上另一個矩陣各元素的新矩陣。
     *
     * @param rhs: 相同大小的矩陣
     *
     * @return 新矩陣
     *
     */
    Matrix operator+(const Matrix &rhs);
    Matrix &operator+=(const Matrix &rhs);

    /*
     * 生成一個由原矩陣各元素減去另一個矩陣各元素的新矩陣。
     *
     * @param rhs: 相同大小的矩陣
     *
     * @return 新矩陣
     *
     */
    Matrix operator-(const Matrix &rhs);
    Matrix &operator-=(const Matrix &rhs);

    /*
     * 生成一個由原矩陣各元素乘上一個常數後的新矩陣。
     *
     * @param rhs: 浮點數常數
     *
     * @return 新矩陣
     *
     */
    Matrix operator*(const double rhs);
    friend Matrix operator*(const double lhs, Matrix &mat) { return mat * lhs; }

    /*
     * 生成一個由原矩陣乘上另一個矩陣的新矩陣。
     *
     * @param rhs: 矩陣，其列需等於原矩陣的行
     *
     * @return 新矩陣
     *
     */
    Matrix operator*(const Matrix &rhs);
    Matrix &operator*=(const double rhs);
    Matrix &operator*=(const Matrix &rhs);

    /*
     * 生成一個由原矩陣各元素除上一個常數後的新矩陣。
     *
     * @param rhs: 浮點數常數
     *
     * @return 新矩陣
     *
     */
    Matrix operator/(const double rhs);
    Matrix &operator/=(const double rhs);

    /*
     * 輸出方法
     *
     */
    friend std::ostream &operator<<(std::ostream &os, const Matrix &mat);

    /*
     * 矩陣內容填入由 Matlab Function 產生的陣列。
     *
     * @param array from Matlab function
     *
     */
    void update_from_matlab(double *arr);

    /*
     * 生成一個轉置矩陣。
     *
     * @return 轉置矩陣
     *
     */
    Matrix gen_Transpose();
};

#endif