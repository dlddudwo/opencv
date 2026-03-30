#pragma once

#include <opencv2/opencv.hpp>
#include <json/json.h>

class COmitProcessOpenCV
{
public:
    cv::Mat Run(const Json::Value& recipe,
                const cv::Mat& imageInput,
                const cv::Mat& selectedRegion,
                cv::Mat* stdevRegionOutput) const;

    cv::Mat GetDefaultRegion(const cv::Mat& imageInput,
                             double thresholdLower,
                             double thresholdUpper) const;

    cv::Mat GetStdDevImage(const Json::Value& recipe,
                           const cv::Mat& imageInput) const;

    cv::Mat GetStdDevFilter(const Json::Value& recipe,
                            const cv::Mat& imageInput,
                            const cv::Mat& stdevImage) const;

    cv::Mat GetStdDevFilterFillup(const Json::Value& recipe,
                                  const cv::Mat& regionInput) const;

    cv::Mat GetEnhanceOmitFilter(const Json::Value& recipe,
                                 const cv::Mat& imageInput,
                                 const cv::Mat& regionTotalRoi,
                                 const cv::Mat& stdevImage,
                                 const cv::Mat& stdevRegionOutput) const;

    cv::Mat GetSelectedRegion(const Json::Value& recipe,
                              const cv::Mat& imageInput,
                              const cv::Mat& regionTotalRoi) const;

    cv::Mat GetEnhanceOmitFilterConcat(const cv::Mat& regionInput,
                                       const cv::Mat& selectedRegion) const;

    cv::Mat GetOmitImage(const cv::Mat& regionTotalRoi,
                         const cv::Mat& imageInput,
                         const cv::Mat& selectedRegion,
                         double resizeFactorVignetting) const;

    cv::Mat GetSharedSelectedRegion(const Json::Value& recipe,
                                    const cv::Mat& imageInput,
                                    const cv::Mat& regionTotalRoi) const;

    cv::Mat GetInspectionRoi(const Json::Value& recipe,
                             const cv::Mat& srcImage) const;

    void CalcOmitDefectData(const cv::Mat& omitInspRoi,
                            const cv::Mat& resultRoi,
                            int& omitResultAreaCount,
                            int& omitResultAreaRate) const;
};
