#include "CvMatRawSerializer.h"

#include <sstream>

namespace hpe
{
    void CvMatRawSerializer::Save(std::string file, cv::Mat &mat)
    {
        std::ofstream stream(file, std::ios_base::out | std::ios_base::binary);
        WriteHeader(stream, mat);
        WriteData(stream, mat);
    }

    void CvMatRawSerializer::WriteHeader(std::ostream &stream, cv::Mat &mat)
    {
        std::stringstream sstream;
        sstream << mat.cols << std::endl
                << mat.rows << std::endl
                << mat.type() << std::endl;
        std::string headerString = sstream.str();
        stream.write(headerString.c_str(), headerString.length());
    }

    void CvMatRawSerializer::WriteData(std::ostream &stream, cv::Mat &mat)
    {
        stream.write((char *)mat.data, mat.dataend - mat.datastart);
    }

    void CvMatRawSerializer::Load(std::string file, cv::Mat &mat)
    {
        std::ifstream stream(file, std::ios_base::in | std::ios_base::binary);
        ReadHeader(stream, mat);
        ReadData(stream, mat);
    }

    void CvMatRawSerializer::ReadHeader(std::istream &stream, cv::Mat &mat)
    {
        int rows;
        int cols;
        int type;
        stream >> cols >> rows >> type;
        mat = cv::Mat(rows, cols, type);
    }

    void CvMatRawSerializer::ReadData(std::istream &stream, cv::Mat &mat)
    {
        char newlineChar;
        stream.read(&newlineChar, 1);
        stream.read((char *)mat.data, mat.dataend - mat.datastart);
    }

    void CvMatRawSerializer::SaveCSVFloat(std::string file, cv::Mat &mat)
    {
        std::ofstream stream(file);
        for (int row = 0;  row < mat.rows; row++)
        {
            for (int col = 0; col < mat.cols; col++)
            {
                stream << mat.at<float>(row, col) << ";";
            }
            stream << std::endl;
        }
    }

}