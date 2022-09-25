#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <iostream>
#include <iomanip>
#include <cmath>
#include <initializer_list>

#define PINV(mat) mat.transpose() * (mat * mat.transpose()).inverse()

enum MatrixType
{
    Diagonal,
    General
};

template <class DATA_TYPE>
class Matrix
{
private:
    DATA_TYPE *matrix;
    unsigned _rows;
    unsigned _cols;

public:
    /*
     * 產生一個矩陣，初始值為0。
     *
     * @param rows: 矩陣列數， cols: 矩陣行數
     *
     */
    Matrix(unsigned rows, unsigned cols);
    Matrix(unsigned rows, unsigned cols, MatrixType type, std::initializer_list<DATA_TYPE> l);
    Matrix(const Matrix &mat);
    ~Matrix();
    Matrix<DATA_TYPE> &operator=(const Matrix<DATA_TYPE> &mat);

    /*
     * 向量取值。
     *
     * @param 元素編號
     *
     */
    const DATA_TYPE &operator[](unsigned num) const;
    DATA_TYPE &operator[](unsigned num);

    /*
     * 矩陣取值。
     *
     * @param row: 元素所在列， col: 元素所在行
     *
     */
    const DATA_TYPE &operator()(unsigned row, unsigned col) const;
    DATA_TYPE &operator()(unsigned row, unsigned col);

    /*
     * 生成一個由原矩陣各元素加上另一個矩陣各元素的新矩陣。
     *
     * @param rhs: 相同大小的矩陣
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator+(const Matrix<DATA_TYPE> &rhs);
    // 不同類型相加
    template <class TYPE>
    Matrix<DATA_TYPE> operator+(const Matrix<TYPE> &rhs)
    {
        if (_rows != rhs.getRow() || _cols != rhs.getCol())
            throw std::logic_error("LHS size is not equal to RHS size.");
        Matrix<DATA_TYPE> ret(_rows, _cols);
        for (unsigned i = 0; i < _rows; i++)
            for (unsigned j = 0; j < _cols; j++)
                ret(i, j) = *(matrix + _cols * i + j) + rhs(i, j);
        return ret;
    }
    template <class TYPE>
    Matrix<DATA_TYPE> operator+(const Matrix<TYPE> &rhs) const
    {
        if (_rows != rhs.getRow() || _cols != rhs.getCol())
            throw std::logic_error("LHS size is not equal to RHS size.");
        Matrix<DATA_TYPE> ret(_rows, _cols);
        for (unsigned i = 0; i < _rows; i++)
            for (unsigned j = 0; j < _cols; j++)
                ret(i, j) = *(matrix + _cols * i + j) + rhs(i, j);
        return ret;
    }
    Matrix<DATA_TYPE> &operator+=(const Matrix<DATA_TYPE> &rhs);

    /*
     * 生成由原矩陣各元素加上負號的新矩陣。
     *
     * @param mat: 矩陣
     *
     * @return 新矩陣
     *
     */
    friend Matrix<DATA_TYPE> operator-(const Matrix<DATA_TYPE> &mat)
    {
        Matrix<DATA_TYPE> ret(mat._rows, mat._cols);
        for (unsigned i = 0; i < mat._rows; i++)
            for (unsigned j = 0; j < mat._cols; j++)
                ret(i, j) = -mat(i, j);
        return ret;
    }

    /*
     * 生成一個由原矩陣各元素減去另一個矩陣各元素的新矩陣。
     *
     * @param rhs: 相同大小的矩陣
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator-(const Matrix<DATA_TYPE> &rhs);
    // 不同類型相減
    template <class TYPE>
    Matrix<DATA_TYPE> operator-(const Matrix<TYPE> &rhs)
    {
        if (_rows != rhs.getRow() || _cols != rhs.getCol())
            throw std::logic_error("LHS size is not equal to RHS size.");
        Matrix<DATA_TYPE> ret(_rows, _cols);
        for (unsigned i = 0; i < _rows; i++)
            for (unsigned j = 0; j < _cols; j++)
                ret(i, j) = *(matrix + _cols * i + j) - rhs(i, j);
        return ret;
    }
    Matrix<DATA_TYPE> &operator-=(const Matrix<DATA_TYPE> &rhs);

    /*
     * 生成一個由原矩陣各元素乘上一個常數後的新矩陣。
     *
     * @param rhs: 常數
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator*(const DATA_TYPE rhs);
    Matrix<DATA_TYPE> operator*(const DATA_TYPE rhs) const;
    friend Matrix<DATA_TYPE> operator*(const DATA_TYPE lhs, Matrix<DATA_TYPE> mat) { return mat * lhs; }
    // 不同類型相乘
    template <class TYPE>
    Matrix<DATA_TYPE> operator*(const TYPE &rhs)
    {
        Matrix<DATA_TYPE> ret(_rows, _cols);
        for (unsigned i = 0; i < _rows; i++)
            for (unsigned j = 0; j < _cols; j++)
                *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) * rhs;
        return ret;
    }
    Matrix<DATA_TYPE> &operator*=(const DATA_TYPE rhs);

    /*
     * 生成一個由原矩陣乘上另一個矩陣的新矩陣。
     *
     * @param rhs: 矩陣，其列需等於原矩陣的行
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator*(const Matrix<DATA_TYPE> &rhs);
    Matrix<DATA_TYPE> operator*(const Matrix<DATA_TYPE> &rhs) const;
    // 不同類型相乘
    template <class TYPE>
    Matrix<DATA_TYPE> operator*(const Matrix<TYPE> &rhs)
    {
        if (_cols != rhs.getRow())
            throw std::logic_error("LHS column is not equal to RHS row.");
        Matrix<DATA_TYPE> ret(_rows, rhs.getCol());
        for (unsigned i = 0; i < _rows; i++)
            for (unsigned j = 0; j < rhs.getCol(); j++)
                for (unsigned k = 0; k < _cols; k++)
                    *(ret.matrix + rhs.getCol() * i + j) += *(matrix + _cols * i + k) * rhs(k, j);
        return ret;
    }
    Matrix<DATA_TYPE> &operator*=(const Matrix<DATA_TYPE> &rhs);

    /*
     * 生成一個由原矩陣各元素除上一個常數後的新矩陣。
     *
     * @param rhs: 常數
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator/(const DATA_TYPE rhs);
    // 不同類型相除
    template <class TYPE>
    Matrix<DATA_TYPE> operator/(const TYPE &rhs)
    {
        Matrix<DATA_TYPE> ret(_rows, _cols);
        for (unsigned i = 0; i < _rows; i++)
            for (unsigned j = 0; j < _cols; j++)
                *(ret.matrix + _cols * i + j) = *(matrix + _cols * i + j) / rhs;
        return ret;
    }
    Matrix<DATA_TYPE> &operator/=(const DATA_TYPE rhs);

    /*
     * 輸出方法
     *
     */
    friend std::ostream &operator<<(std::ostream &os, const Matrix<DATA_TYPE> &mat)
    {
        for (unsigned i = 0; i < mat._rows; i++)
        {
            for (unsigned j = 0; j < mat._cols; j++)
                std::cout << std::right << std::setw(15) << std::fixed << mat(i, j);
            std::cout << std::endl;
        }
        return os;
    }

    /*
     * 矩陣內容填入由 Matlab Function 產生的陣列。
     *
     * @param array from Matlab function
     *
     */
    void update_from_matlab(const DATA_TYPE *arr);

    /*
     * 生成一個轉置矩陣。
     *
     * @return 轉置矩陣
     *
     */
    Matrix<DATA_TYPE> transpose();
    Matrix<DATA_TYPE> transpose() const;

    /*
     * 生成一個反矩陣。
     *
     * @return 反矩陣
     *
     */
    Matrix<double> inverse();

    /*
     * 取得矩陣列數。
     *
     * @return 矩陣列數
     *
     */
    const unsigned getRow() const { return _rows; }

    /*
     * 取得矩陣行數。
     *
     * @return 矩陣行數
     *
     */
    const unsigned getCol() const { return _cols; }
};

/*
 * 計算方陣行列式值。
 *
 * @return 方陣行列式值
 *
 */
template <class DATA_TYPE>
DATA_TYPE det(const Matrix<DATA_TYPE> &mat)
{
    if (mat.getRow() != mat.getCol())
        throw std::logic_error("Input must be square matrix.");
    if (mat.getRow() == 2)
        return mat(0, 0) * mat(1, 1) - mat(0, 1) * mat(1, 0);
    else if (mat.getRow() == 1)
        return mat(0, 0);
    DATA_TYPE detVal = 0;
    Matrix<DATA_TYPE> minorMat(mat.getRow() - 1, mat.getRow() - 1);
    for (unsigned k = 0; k < mat.getRow(); k++)
    {
        for (unsigned i = 1, mat_i = 0; i < mat.getRow(); i++, mat_i++)
        {
            for (unsigned j = 0, mat_j = 0; j < mat.getRow(); j++)
            {
                if (j == k)
                    continue;
                minorMat(mat_i, mat_j++) = mat(i, j);
            }
        }
        if (k % 2 == 0)
            detVal += mat(0, k) * det(minorMat);
        else
            detVal -= mat(0, k) * det(minorMat);
    }
    return detVal;
}

/*
 * 計算向量 Euclidean norm。
 *
 * @return Euclidean norm 數值
 *
 */
template <class DATA_TYPE>
double norm(const Matrix<DATA_TYPE> &mat)
{
    if (mat.getRow() != 1 && mat.getCol() != 1)
        throw std::logic_error("Input must be a vector.");
    double norm = 0;
    for (unsigned i = 0; i < mat.getRow(); i++)
        for (unsigned j = 0; j < mat.getCol(); j++)
            norm += mat(i, j) * mat(i, j);
    return sqrt(norm);
}
#endif
