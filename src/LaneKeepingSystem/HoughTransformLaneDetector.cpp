// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file HoughTransformLaneDetector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief hough transform lane detector class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <numeric>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"].as<PREC>();
    mHoughThreshold = config["HOUGH"]["THRESHOLD"].as<int32_t>();
    mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"].as<int32_t>();
    mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::pair<PREC, PREC> HoughTransformLaneDetector<PREC>::getLineParameters(const Lines& lines, const Indices& lineIndices) {

    // TODO : Implement this function

    // 기존 코드의 vector<Vec4i>& lines의 의미 : lines vs lineIndices -> 잘 이해가 안됨.. 왜 lineIndices인지..?
    uint32_t numLines = static_cast<uint32_t>(lineIndices.size());

    if (numLines == 0)
        return { 0.0f, 0.0f };

    int32_t = x1;
    int32_t = x2;
    int32_t = y1;
    int32_t = y2;

    // xSum, ySum이 int형인데.. 우리는 float으로 가져와서 PREC 선언
    PREC xSum = 0.0f;
    PREC ySum = 0.0f;
    PREC mSum = 0.0f;

    PREC xAvg = 0.0f;
    PREC yAvg = 0.0f;
    PREC m = 0.0f;
    PREC b = 0.0f;
		
		// 성능과 안전성 위해 const auto& 사용
    for (const auto& lineIndex : lineIndices) {
        // x1, x2, y1, y2 선언 -> HoughTransformLaneDetector.hpp 에서 0~4(기존 코드) -> x1~y2 정의
        x1 = lines[lineIndex][HoughIndex::x1];
        y1 = lines[lineIndex][HoughIndex::y1];
        x2 = lines[lineIndex][HoughIndex::x2];
        y2 = lines[lineIndex][HoughIndex::y2];
        xSum += x1 + x2;
        ySum += y1 + y2;
        mSum += static_cast<PREC>(y2 - y1) / static_cast<PREC>(X2 - X1);
    }
    // numLines는 unsigned int이기 때문에 static_cast로 형 변환
    xAvg = xSum / static_cast<PREC>(numLines * 2);
    yAvg = ySum / static_cast<PREC>(numLines * 2);

    // 최종 m, b 구하기
    m = mSum / static_cast<PREC>(numLines);
    b = yAvg - m * xAvg;

    return { m, b };
}

template <typename PREC>
int32_t HoughTransformLaneDetector<PREC>::getLinePositionX(const Lines& lines, const Indices& lineIndices, Direction direction)
{
    // TODO : Implement this function
    int32_t positionX = 0;

    return positionX;
}

template <typename PREC>
std::pair<Indices, Indices> HoughTransformLaneDetector<PREC>::divideLines(const Lines& lines)
    {
        // TODO : Implement this function
        Indices leftLineIndices;
        Indices rightLineIndices;
        std::vector<float> slopes;
        Lines new_lines;
        float slope;
        Line line;
        
        for (int i = 0; i < linesSize; ++i)
        {
            line = lines[i];
            x1 = line[0], y1 = line[1];
            x2 = line[2], y2 = line[3];
            if (x2 - x1 == 0)
            {
                slope = 0.0;
            }
            else 
            {
                slope = float(y2 - y1) / float(x2 - x1);
            }
            if (std::abs(slope) > 0 && std::(abs(slope) <= mHoughLineSlopeRange)
            {
                slopes.push_back(slope);
                new_lines.push_back(line);
            }
        }

        Lines left_lines, right_lines;
        //std::vector<float> left_lines_slope, right_lines_slope;
        for (int i = 0; i < new_lines.size(); ++i)
            {
                line = new_lines[i];
                slope = slopes[i];
                x1 = line[0], y1 = line[1];
                x2 = line[2], y2 = line[3];

                if((slope < 0 ) && (x2 < 320))
                {
                    
                    leftLineIndices.push_back(line);

                    
                }
                else if ((slope > 0) && (x1 > 320))
                {
                    rightLineIndices.push_back(line);
                }
        }
        

        return { leftLineIndices, rightLineIndices };
    }

template <typename PREC>
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
{
    // TODO : Implement this function
    int32_t leftPositionX = 0;
    int32_t rightPositionX = 0;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blur;
    cv::GaussianBlur(gray, blur, cv::Size(), 1.);
    // cv::imshow("blur", blur);

    cv::Mat canny;
    cv::Canny(blur, canny, 150, 200);

    cv::Mat roi;

    roi = Canny(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));

    // roi houghlineP
    Lines houghLines;
    cv::HoughLinesP(roi, houghLines, 1, CV_PI/180, 45, 45, 20);

    if (houghLines.empty())
        return {0, mImageWidth};

    std::pair<Indices, Indices> leftRightLinesPair;
    leftRightLinesPair = HoughTransformLaneDetector::divideLines(houghLines);

    Indices leftLines = leftRightLinesPair.first;
    Indices rightLInes = leftRightLinesPair.second;

    leftPositionX = getLinePositionX(houghLines, leftLines, Direction::LEFT);
    rightPositionX = getLinePositionX(houghLines, rightLines, Direction::RIGHT);


    if (mDebugging)
        // draw parts
        image.copyTo(mDebugFrame);

    return { leftPositionX, rightPositionX };
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices)
{
    auto draw = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();

            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, { b, g, r }, kDebugLineWidth);
        }
    };

    draw(lines, leftLineIndices);
    draw(lines, rightLineIndices);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(leftPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(rightPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(estimatedPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(estimatedPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kRed, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(mImageWidth / 2 - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(mImageWidth / 2 + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kBlue, kDebugLineWidth);
}

template class HoughTransformLaneDetector<float>;
template class HoughTransformLaneDetector<double>;
} // namespace Xycar
