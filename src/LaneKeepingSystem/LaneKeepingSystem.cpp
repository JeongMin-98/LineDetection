// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class source file
 * @version 1.1
 * @date 2023-05-02
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    curvePID = new PIDController<PREC>(config["PID"]["CURVE_P_GAIN"].as<PREC>(), config["PID"]["CURVE_I_GAIN"].as<PREC>(), config["PID"]["CURVE_D_GAIN"].as<PREC>());
    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mHoughTransformLaneDetector = new HoughTransformLaneDetector<PREC>(config);
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mCteParams = config["CTE"]["CTE_ERROR"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mPID;
    delete curvePID;
    delete mMovingAverage;
    delete mHoughTransformLaneDetector;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::sender(bool leftDetector, bool rightDetector, DetectorPtr ptr)
{
    ptr->oneLaneByLeft = leftDetector;
    ptr->oneLaneByRight = rightDetector;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    bool leftDetector;
    bool rightDetector;

    PREC leftCurveCTE = 0.0f;
    int32_t leftCount = 0;
    PREC rightCurveCTE = 0.0f;
    int32_t rightCount = 0;
    PREC forwardCTE;
    int32_t forwardCount = 0;

    ros::Rate rate(kFrameRate);
    while (ros::ok())
    {
        ros::spinOnce();
        if (mFrame.empty())
            continue;
        cv::imshow("frame", mFrame);
        const auto [leftPositionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame);

        // mMovingAverage->addSample(static_cast<int32_t>((leftPositionX + rightPositionX) / 2));

        int32_t estimatedPositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);

        /**
         * below code Written By JeongMin (jeongmin981@gmail.com)
         * Line 96 ~ 154
         */
        int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
        
        PREC cte = (static_cast<PREC>(errorFromMid) * 2.0f) / static_cast<PREC>(mFrame.cols);
        // PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));

        /** errorFromMid의 값이 양의 값일 때 특정 값보다 큰 경우 
             => HoughTransformLaneDetector에게 왼쪽에서만 차선 검출하라고 지시.
            만약 왼쪽 차선의 추출이 불가능하면 다시 양쪽의 경우도 검출해본다.

            errorFromMid의 값이 음의 값인데 특정 음의 값보다 큰 경우 (즉 작은 경우)
            회전을 왼쪽으로 한다고 판단 Detector에게 오른쪽에서만 차선 검출하라고 지시
        **/
        if (mDebugging)

            if (!leftDetector || !rightDetector)
            {
                std::cout << "leftOnly: " << leftDetector;
                std::cout << "rightOnly: " << rightDetector;
                std::cout << "cte: " << cte << std::endl;
            }

        if (cte > mCteParams)
        {
            if (mDebugging)
            {
                rightCount += 1;
                rightCurveCTE += cte;
            }
            leftDetector = true;
            rightDetector = false;
            errorFromMid = static_cast<PREC>(curvePID->getControlOutput(errorFromMid));
        }
        else if (cte < mCteParams * -1.0f)
        {
            if (mDebugging)
            {
                leftCount += 1;
                leftCurveCTE += cte;
            }
            leftDetector = false;
            rightDetector = true;
            errorFromMid = static_cast<PREC>(curvePID->getControlOutput(errorFromMid));
        }
        else if (abs(cte) <= mCteParams)
        {
            if (mDebugging)
            {
                forwardCount += 1;
                forwardCTE += cte;
            }
            leftDetector = true;
            rightDetector = true;
            errorFromMid = static_cast<PREC>(mPID->getControlOutput(errorFromMid));
        }


        speedControl(errorFromMid);
        drive(errorFromMid);
        sender(leftDetector, rightDetector, mHoughTransformLaneDetector);

        if (mDebugging)
        {
            
            std::cout << "lpos: " << leftPositionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << std::endl;
            mHoughTransformLaneDetector->drawRectangles(leftPositionX, rightPositionX, estimatedPositionX);
            cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            cv::imshow("roi", mHoughTransformLaneDetector->getDebugROI());
            cv::waitKey(1);
        }
        // rate.sleep();
    }

    if (mDebugging)
    {
        PREC averageRightCTE = rightCurveCTE / static_cast<PREC>(rightCount);
        PREC averageLeftCTE = leftCurveCTE / static_cast<PREC>(leftCount);
        PREC averageForwardCTE = forwardCTE / static_cast<PREC>(forwardCount);
        std::cout << "result: " << std::endl;
        std::cout << "right: " << averageRightCTE << std::endl;
        std::cout << "left: " << averageLeftCTE  << std::endl;
        std::cout << "forward: " << averageForwardCTE <<std::endl;
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);
    // motorMessage.speed = std::round(6.0f);
    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
