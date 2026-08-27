/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   矩阵/向量计算库（基于 CMSIS-DSP 加速）
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "arm_math.h"   // CMSIS-DSP 矩阵函数

#pragma once

// 模板矩阵类：行数 _rows，列数 _cols（编译期常量）
template <int _rows, int _cols>
class Matrixf
{
public:
    // ---------- 构造函数 ----------

    /**
     * @brief 无参构造：数据初始化为 0，并初始化 CMSIS-DSP 矩阵实例
     * @note  使用 constexpr 修饰，但在 C++20 之前调用 C 库函数并不真正编译期计算，此处仅为标记
     */
    constexpr Matrixf(void) : rows_(_rows), cols_(_cols) { 
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_); 
    }

    /**
     * @brief 从一维数组构造（数据按行主序存储）
     * @param data 长度为 _rows*_cols 的 float 数组
     */
    constexpr Matrixf(float data[_rows * _cols]) : Matrixf() {
        memcpy(this->data_, data, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief 拷贝构造函数
     * @param mat 同类型的矩阵
     */
    constexpr Matrixf(const Matrixf<_rows, _cols> &mat) : Matrixf() {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief 析构函数（无特殊资源释放，所有数据在栈上）
     */
    ~Matrixf(void) {}

    // ---------- 属性查询 ----------
    uint32_t rows(void) const { return _rows; }   // 行数
    uint32_t cols(void) const { return _cols; }   // 列数

    // ---------- 元素访问 ----------
    /**
     * @brief 重载下标运算符，返回第 row 行的指针（行主序）
     * @param row 行索引（从 0 开始）
     */
    float *operator[](const int &row) { return &this->data_[row * _cols]; }

    // ---------- 赋值运算符 ----------
    /**
     * @brief 拷贝赋值
     * @param mat 右侧矩阵
     */
    Matrixf<_rows, _cols> &operator=(const Matrixf<_rows, _cols> mat) {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        return *this;
    }

    // ---------- 复合赋值运算符（原地运算） ----------
    /**
     * @brief 矩阵加法（自身 += 其他矩阵）
     * @note  使用 CMSIS-DSP 的 arm_mat_add_f32
     * @warning 输入和输出指针相同，CMSIS 标准实现支持原地操作，但某些编译优化可能产生别名问题，建议谨慎。
     */
    Matrixf<_rows, _cols> &operator+=(const Matrixf<_rows, _cols> mat) {
        arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 矩阵减法（自身 -= 其他矩阵）
     */
    Matrixf<_rows, _cols> &operator-=(const Matrixf<_rows, _cols> mat) {
        arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 标量乘法（自身 *= 标量）
     */
    Matrixf<_rows, _cols> &operator*=(const float &val) {
        arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 标量除法（自身 /= 标量）
     */
    Matrixf<_rows, _cols> &operator/=(const float &val) {
        arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
        return *this;
    }

    // ---------- 二元运算符（返回新矩阵） ----------
    /**
     * @brief 矩阵加法（返回新矩阵）
     */
    Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols> &mat) const {
        Matrixf<_rows, _cols> res;
        arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵减法（返回新矩阵）
     */
    Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols> &mat) const {
        Matrixf<_rows, _cols> res;
        arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 右标量乘（矩阵 * 标量）
     */
    Matrixf<_rows, _cols> operator*(const float &val) const {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 左标量乘（标量 * 矩阵），友元函数
     */
    friend Matrixf<_rows, _cols> operator*(const float &val, const Matrixf<_rows, _cols> &mat) {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 标量除法（矩阵 / 标量）
     */
    Matrixf<_rows, _cols> operator/(const float &val) const {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵乘法（要求左列数 == 右行数）
     * @tparam cols2 右侧矩阵的列数
     * @note  返回值类型为 Matrixf<_rows, cols2>
     */
    template <int cols2>
    friend Matrixf<_rows, cols2> operator*(const Matrixf<_rows, _cols> &mat1, const Matrixf<_cols, cols2> &mat2) {
        Matrixf<_rows, cols2> res;
        arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
        return res;
    }

    // ---------- 比较运算符 ----------
    /**
     * @brief 判断两个矩阵是否完全相等（逐元素比较，使用 `!=`，易受浮点误差影响）
     */
    bool operator==(const Matrixf<_rows, _cols> &mat) const {
        for (int i = 0; i < _rows * _cols; i++) {
            if (this->data_[i] != mat.data_[i])
                return false;
        }
        return true;
    }

    // ---------- 子矩阵操作 ----------
    /**
     * @brief 提取子矩阵（块）
     * @tparam rows   子矩阵行数
     * @tparam cols   子矩阵列数
     * @param start_row  起始行
     * @param start_col  起始列
     * @return 新的子矩阵
     */
    template <int rows, int cols>
    Matrixf<rows, cols> block(const int &start_row, const int &start_col) const {
        Matrixf<rows, cols> res;
        for (int row = start_row; row < start_row + rows; row++) {
            // 拷贝行数据
            memcpy((float *)res[0] + (row - start_row) * cols, 
                   (float *)this->data_ + row * _cols + start_col, 
                   cols * sizeof(float));
        }
        return res;
    }

    /**
     * @brief 获取指定行（返回 1×_cols 矩阵）
     */
    Matrixf<1, _cols> row(const int &row) const { return block<1, _cols>(row, 0); }

    /**
     * @brief 获取指定列（返回 _rows×1 矩阵）
     */
    Matrixf<_rows, 1> col(const int &col) const { return block<_rows, 1>(0, col); }

    // ---------- 矩阵运算 ----------
    /**
     * @brief 转置
     */
    Matrixf<_cols, _rows> trans(void) const {
        Matrixf<_cols, _rows> res;
        arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 迹（对角线元素之和）
     */
    float trace(void) const {
        float res = 0;
        for (int i = 0; i < fmin(_rows, _cols); i++) {
            res += (*this)[i][i];
        }
        return res;
    }

    /**
     * @brief 矩阵的欧几里得范数（Frobenius 范数）
     */
    float norm(void) const { 
        return sqrtf((this->trans() * *this)[0][0]); 
    }

    /**
     * @brief 求逆（仅方阵）
     * @return 逆矩阵，若奇异则返回零矩阵
     */
    Matrixf<_cols, _rows> inv(void) const {
        if (_cols != _rows)
            return Matrixf<_cols, _rows>::zeros();

        Matrixf<_cols, _rows> res;
        arm_status status = arm_mat_inverse_f32(&this->arm_mat_, &res);

        if (status == ARM_MATH_SINGULAR)
            return Matrixf<_cols, _rows>::zeros();

        return res;
    }

    // ---------- 静态工厂方法 ----------
    /**
     * @brief 返回一个全零矩阵
     */
    static Matrixf<_rows, _cols> zeros(void) {
        float data[_rows * _cols] = {0};
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief 返回一个全一矩阵
     */
    static Matrixf<_rows, _cols> ones(void) {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < _rows * _cols; i++) {
            data[i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief 返回单位矩阵（主对角线为 1，其余为 0）
     * @note  若行列不等，则只填充 min(_rows,_cols) 个 1，其余为 0（非方阵情况下也称为“广义单位阵”）
     */
    static Matrixf<_rows, _cols> eye(void) {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < fmin(_rows, _cols); i++) {
            data[i * _cols + i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief 由列向量构造对角矩阵
     * @param vec 长度为 _rows 的列向量（若 _rows != _cols，只取前 min(_rows,_cols) 个）
     */
    static Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
        Matrixf<_rows, _cols> res = Matrixf<_rows, _cols>::zeros();
        for (int i = 0; i < fmin(_rows, _cols); i++) {
            res[i][i] = vec[i][0];
        }
        return res;
    }

public:
    arm_matrix_instance_f32 arm_mat_;   // CMSIS-DSP 矩阵实例（包含行数、列数、数据指针）

protected:
    int rows_, cols_;                   // 冗余存储的行列数（与模板参数重复，占用 8 字节）
    float data_[_rows * _cols];         // 矩阵数据（栈上分配）
};