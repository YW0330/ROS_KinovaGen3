#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <iostream>

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
    Matrix<DATA_TYPE> &operator+=(const Matrix<DATA_TYPE> &rhs);

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
     * @param rhs: 浮點數常數
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator*(const DATA_TYPE rhs);
    friend Matrix<DATA_TYPE> operator*(const DATA_TYPE lhs, Matrix<DATA_TYPE> &mat) { return mat * lhs; }
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
    Matrix<DATA_TYPE> &operator*=(const Matrix<DATA_TYPE> &rhs);

    /*
     * 生成一個由原矩陣各元素除上一個常數後的新矩陣。
     *
     * @param rhs: 浮點數常數
     *
     * @return 新矩陣
     *
     */
    Matrix<DATA_TYPE> operator/(const DATA_TYPE rhs);
    Matrix<DATA_TYPE> &operator/=(const DATA_TYPE rhs);

    /*
     * 輸出方法
     *
     */
    friend std::ostream &operator<<(std::ostream &os, const Matrix &mat)
    {
        for (unsigned i = 0; i < mat._rows; i++)
        {
            for (unsigned j = 0; j < mat._cols; j++)
                std::cout << *(mat.matrix + mat._cols * i + j) << "\t";
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
    void update_from_matlab(DATA_TYPE *arr);

    /*
     * 生成一個轉置矩陣。
     *
     * @return 轉置矩陣
     *
     */
    Matrix<DATA_TYPE> Transpose();

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

#endif
