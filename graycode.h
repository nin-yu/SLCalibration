#ifndef GRAYCODE_H
#define GRAYCODE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

class GrayCode
{
public:
    GrayCode(int n = 3);
    ~GrayCode();

    int m_n;
    cv::Mat m_codes;                // 格雷码矩阵 (n, 2^n)
    std::map<int, int> m_code2k_int; // 整数索引的快速查找
    std::map<int, int> m_k2v;

private:
    std::vector<std::string> __createGrayCode(int n);
    void __formCodes(int n, const std::vector<std::string>& code_temp);
    std::string __code2k(int k);
    int __k2v(int k);
};

#endif // GRAYCODE_H
