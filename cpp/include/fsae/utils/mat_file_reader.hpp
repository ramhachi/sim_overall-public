#pragma once
/// @file mat_file_reader.hpp
/// @brief MATファイル読み込みユーティリティ（NN Tire Model Lookup Table用）

#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include <matio.h>

namespace fsae::utils {

/// 5次元配列（Lookup Table用）
template<typename T>
class Array5D {
public:
    Array5D() = default;
    
    Array5D(size_t d0, size_t d1, size_t d2, size_t d3, size_t d4)
        : dims_{d0, d1, d2, d3, d4} {
        data_.resize(d0 * d1 * d2 * d3 * d4);
    }
    
    /// 5次元インデックスアクセス
    T& operator()(size_t i0, size_t i1, size_t i2, size_t i3, size_t i4) {
        size_t idx = compute_index(i0, i1, i2, i3, i4);
        return data_[idx];
    }
    
    const T& operator()(size_t i0, size_t i1, size_t i2, size_t i3, size_t i4) const {
        size_t idx = compute_index(i0, i1, i2, i3, i4);
        return data_[idx];
    }
    
    const std::array<size_t, 5>& dimensions() const { return dims_; }
    size_t size() const { return data_.size(); }
    const std::vector<T>& data() const { return data_; }
    std::vector<T>& data() { return data_; }
    
private:
    std::array<size_t, 5> dims_;
    std::vector<T> data_;
    
    size_t compute_index(size_t i0, size_t i1, size_t i2, size_t i3, size_t i4) const {
        // MATLAB列優先（column-major）からC++行優先（row-major）への変換
        // MATLAB: (i0, i1, i2, i3, i4) → C++での格納順序
        return i0 + dims_[0] * (i1 + dims_[1] * (i2 + dims_[2] * (i3 + dims_[3] * i4)));
    }
};

/// MATファイル読み込みクラス
class MatFileReader {
public:
    MatFileReader() = default;
    ~MatFileReader() = default;
    
    /// MATファイルを開く
    /// @param filepath MATファイルのパス
    /// @throws std::runtime_error ファイルが開けない場合
    static mat_t* open(const std::string& filepath) {
        mat_t* matfp = Mat_Open(filepath.c_str(), MAT_ACC_RDONLY);
        if (!matfp) {
            throw std::runtime_error("Failed to open MAT file: " + filepath);
        }
        return matfp;
    }
    
    /// MATファイルを閉じる
    static void close(mat_t* matfp) {
        if (matfp) {
            Mat_Close(matfp);
        }
    }
    
    /// 1次元double配列を読み込み
    /// @param matfp MATファイルポインタ
    /// @param varname 変数名
    /// @return 読み込んだデータ
    /// @throws std::runtime_error 変数が見つからない、型が不正な場合
    static std::vector<double> read_array_1d(mat_t* matfp, const std::string& varname) {
        matvar_t* var = Mat_VarRead(matfp, varname.c_str());
        if (!var) {
            throw std::runtime_error("Variable not found: " + varname);
        }
        
        if (var->class_type != MAT_C_DOUBLE || var->isComplex) {
            Mat_VarFree(var);
            throw std::runtime_error("Variable is not a real double array: " + varname);
        }
        
        size_t numel = 1;
        for (int i = 0; i < var->rank; i++) {
            numel *= var->dims[i];
        }
        
        double* data_ptr = static_cast<double*>(var->data);
        std::vector<double> result(data_ptr, data_ptr + numel);
        
        Mat_VarFree(var);
        return result;
    }
    
    /// 5次元double配列を読み込み
    /// @param matfp MATファイルポインタ
    /// @param varname 変数名
    /// @param expected_dims 期待される次元（検証用、空の場合はスキップ）
    /// @return 読み込んだ5次元配列
    /// @throws std::runtime_error 変数が見つからない、次元が不正な場合
    static Array5D<double> read_array_5d(
        mat_t* matfp, 
        const std::string& varname,
        const std::array<size_t, 5>& expected_dims = {}) 
    {
        matvar_t* var = Mat_VarRead(matfp, varname.c_str());
        if (!var) {
            throw std::runtime_error("Variable not found: " + varname);
        }
        
        if (var->class_type != MAT_C_DOUBLE || var->isComplex) {
            Mat_VarFree(var);
            throw std::runtime_error("Variable is not a real double array: " + varname);
        }
        
        if (var->rank != 5) {
            Mat_VarFree(var);
            throw std::runtime_error("Variable is not 5-dimensional: " + varname);
        }
        
        // 次元の取得
        std::array<size_t, 5> dims;
        for (int i = 0; i < 5; i++) {
            dims[i] = var->dims[i];
        }
        
        // 期待次元のチェック
        if (expected_dims[0] != 0) {  // 0でない場合は検証
            for (int i = 0; i < 5; i++) {
                if (dims[i] != expected_dims[i]) {
                    Mat_VarFree(var);
                    throw std::runtime_error("Dimension mismatch for variable: " + varname);
                }
            }
        }
        
        // データのコピー
        Array5D<double> result(dims[0], dims[1], dims[2], dims[3], dims[4]);
        double* data_ptr = static_cast<double*>(var->data);
        
        // MATLABの列優先からC++の行優先への変換
        size_t total_size = dims[0] * dims[1] * dims[2] * dims[3] * dims[4];
        for (size_t flat_idx = 0; flat_idx < total_size; flat_idx++) {
            // MATLABの列優先インデックスから各次元のインデックスを計算
            size_t temp = flat_idx;
            size_t i0 = temp % dims[0]; temp /= dims[0];
            size_t i1 = temp % dims[1]; temp /= dims[1];
            size_t i2 = temp % dims[2]; temp /= dims[2];
            size_t i3 = temp % dims[3]; temp /= dims[3];
            size_t i4 = temp;
            
            result(i0, i1, i2, i3, i4) = data_ptr[flat_idx];
        }
        
        Mat_VarFree(var);
        return result;
    }
    
    /// スカラー値を読み込み
    /// @param matfp MATファイルポインタ
    /// @param varname 変数名
    /// @return 読み込んだスカラー値
    static double read_scalar(mat_t* matfp, const std::string& varname) {
        auto vec = read_array_1d(matfp, varname);
        if (vec.size() != 1) {
            throw std::runtime_error("Variable is not a scalar: " + varname);
        }
        return vec[0];
    }
};

/// RAII wrapper for MAT file
class MatFile {
public:
    explicit MatFile(const std::string& filepath) 
        : matfp_(MatFileReader::open(filepath)) {}
    
    ~MatFile() {
        MatFileReader::close(matfp_);
    }
    
    // コピー禁止
    MatFile(const MatFile&) = delete;
    MatFile& operator=(const MatFile&) = delete;
    
    // ムーブ可能
    MatFile(MatFile&& other) noexcept : matfp_(other.matfp_) {
        other.matfp_ = nullptr;
    }
    
    mat_t* get() const { return matfp_; }
    
private:
    mat_t* matfp_;
};

} // namespace fsae::utils
