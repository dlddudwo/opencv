#include <opencv2/opencv.hpp>
#include <json/json.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace {

enum RCP_OMIT_PROCESS
{
    DILATION = 0,
    DILATION_STDEV,
    INSP_ROI_MAX_THRESHOLD,
    INSP_ROI_MIN_THRESHOLD,
    ORIGINAL_THRESHOLD,
    PTN_BTH,
    PTN_WTH,
    SHARED_BOLB_MIN_AREA,
    SKELETON_POINTS,
    SKELETON_POINTS_MIN,
    STDEV_FILTER_MASK_SIZE,
    STDEV_FILTER_SCALE,
    STDEV_THRESHOLD,
    STICKER_AREA_MAX,
    STICKER_AREA_MIN,
    STICKER_CLOSING,
    STICKER_DILATION,
    STICKER_HOLES_NUM_MIN,
    WEAKSC_FLAG,
    WEAKSC_LOWER_TH,
    WEAKSC_MIN_LENGTH_FINAL,
    WEAKSC_MIN_LENGTH_SPLIT,
    WEAKSC_MIN_UNION_LENGTH,
    WEAKSC_MIN_UNION_MAXSHIFT,
    WEAKSC_MIN_UNION_RADIAN,
    WEAKSC_RESIZE,
    WEAKSC_UPPER_TH,
    FILLUP_MODE,
    FILLUP_AREA,
    WEAKSC_USE,
    STICKER_USE,
    DYNAMIC_FILTER_SIZE,
    DYNAMIC_THRESHOLD,
    SHARED_BLOB_MIN_AREA
};

inline int oddKernel(double radius)
{
    const int r = std::max(0, static_cast<int>(std::lround(radius)));
    return std::max(1, r * 2 + 1);
}

cv::Mat ensureGray8(const cv::Mat& src)
{
    if (src.empty())
        return {};

    cv::Mat gray;
    if (src.channels() == 1)
        gray = src;
    else
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    if (gray.type() != CV_8U)
        gray.convertTo(gray, CV_8U);
    return gray;
}

cv::Mat ensureMask8(const cv::Mat& src)
{
    cv::Mat mask = ensureGray8(src);
    if (mask.empty())
        return mask;
    cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);
    return mask;
}

cv::Mat thresholdRange(const cv::Mat& srcGray, double low, double high)
{
    cv::Mat mask;
    cv::inRange(srcGray, cv::Scalar(low), cv::Scalar(high), mask);
    return mask;
}

cv::Mat selectByArea(const cv::Mat& mask, int minArea, int maxArea)
{
    cv::Mat bw = ensureMask8(mask);
    if (bw.empty())
        return bw;

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(bw, labels, stats, centroids, 8, CV_32S);

    cv::Mat out = cv::Mat::zeros(mask.size(), CV_8U);
    for (int i = 1; i < n; ++i)
    {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < minArea || area > maxArea)
            continue;
        out.setTo(255, labels == i);
    }
    return out;
}

cv::Mat morphCircle(const cv::Mat& mask, int op, double radius)
{
    cv::Mat bw = ensureMask8(mask);
    const int k = oddKernel(radius);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::Mat out;
    cv::morphologyEx(bw, out, op, kernel);
    return out;
}

cv::Mat dilateCircle(const cv::Mat& mask, double radius)
{
    return morphCircle(mask, cv::MORPH_DILATE, radius);
}

cv::Mat erodeCircle(const cv::Mat& mask, double radius)
{
    return morphCircle(mask, cv::MORPH_ERODE, radius);
}

cv::Mat fillAllHoles(const cv::Mat& mask)
{
    cv::Mat bw = ensureMask8(mask);
    if (bw.empty())
        return bw;

    cv::Mat flood = bw.clone();
    cv::floodFill(flood, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat floodInv;
    cv::bitwise_not(flood, floodInv);
    cv::Mat filled = bw | floodInv;
    return filled;
}

cv::Mat fillHolesByArea(const cv::Mat& mask, double maxHoleArea)
{
    cv::Mat bw = ensureMask8(mask);
    if (bw.empty())
        return bw;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw.clone(), contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat out = bw.clone();
    for (size_t i = 0; i < contours.size(); ++i)
    {
        const int parent = hierarchy[i][3];
        if (parent < 0)
            continue; // outer contour only

        const double area = std::abs(cv::contourArea(contours[i]));
        if (area <= maxHoleArea)
            cv::drawContours(out, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
    }
    return out;
}

cv::Mat unionMask(const cv::Mat& a, const cv::Mat& b)
{
    if (a.empty())
        return ensureMask8(b);
    if (b.empty())
        return ensureMask8(a);

    cv::Mat aa = ensureMask8(a), bb = ensureMask8(b), out;
    cv::bitwise_or(aa, bb, out);
    return out;
}

cv::Mat intersectionMask(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat aa = ensureMask8(a), bb = ensureMask8(b), out;
    cv::bitwise_and(aa, bb, out);
    return out;
}

cv::Mat reduceDomain(const cv::Mat& srcGray, const cv::Mat& roiMask)
{
    cv::Mat out = cv::Mat::zeros(srcGray.size(), srcGray.type());
    srcGray.copyTo(out, ensureMask8(roiMask));
    return out;
}

double compRatio(const cv::Rect& r)
{
    if (r.height <= 0)
        return std::numeric_limits<double>::infinity();
    return static_cast<double>(r.width) / static_cast<double>(r.height);
}

int childHoleCount(int contourIdx, const std::vector<cv::Vec4i>& hierarchy)
{
    int count = 0;
    for (size_t i = 0; i < hierarchy.size(); ++i)
    {
        if (hierarchy[i][3] == contourIdx)
            ++count;
    }
    return count;
}

} // namespace

class COmitProcessOpenCV
{
public:
    cv::Mat Run(const Json::Value& recipe,
                const cv::Mat& imageInput,
                const cv::Mat& selectedRegion,
                cv::Mat* stdevRegionOutput) const
    {
        const cv::Mat stdevImage = GetStdDevImage(recipe, imageInput);
        const cv::Mat stdevFilterImage = GetStdDevFilter(recipe, imageInput, stdevImage);

        cv::Mat stdevFilled = GetStdDevFilterFillup(recipe, stdevFilterImage);
        if (stdevRegionOutput)
            *stdevRegionOutput = stdevFilled;

        return GetEnhanceOmitFilter(recipe, imageInput, selectedRegion, stdevImage, stdevFilled);
    }

    cv::Mat GetDefaultRegion(const cv::Mat& imageInput, double thresholdLower, double thresholdUpper) const
    {
        const cv::Mat gray = ensureGray8(imageInput);
        cv::Mat low = thresholdRange(gray, 0.0, thresholdLower);
        cv::Mat high = thresholdRange(gray, thresholdUpper, 255.0);
        return fillAllHoles(unionMask(low, high));
    }

    cv::Mat GetStdDevImage(const Json::Value& recipe, const cv::Mat& imageInput) const
    {
        const cv::Mat gray = ensureGray8(imageInput);
        const int filterSize = std::max(1, recipe[STDEV_FILTER_MASK_SIZE].asInt());
        const cv::Size ksize(filterSize | 1, filterSize | 1);

        cv::Mat f32;
        gray.convertTo(f32, CV_32F);

        cv::Mat mean, sqr, meanSqr, var, stddev;
        cv::blur(f32, mean, ksize);
        cv::multiply(f32, f32, sqr);
        cv::blur(sqr, meanSqr, ksize);
        var = meanSqr - mean.mul(mean);
        cv::max(var, 0, var);
        cv::sqrt(var, stddev);

        const double scale = recipe[STDEV_FILTER_SCALE].asDouble();
        cv::Mat scaled;
        stddev.convertTo(scaled, CV_8U, scale, 0.0);
        return scaled;
    }

    cv::Mat GetStdDevFilter(const Json::Value& recipe,
                            const cv::Mat& imageInput,
                            const cv::Mat& stdevImage) const
    {
        const cv::Mat gray = ensureGray8(imageInput);
        cv::Mat stdevMask = thresholdRange(ensureGray8(stdevImage), recipe[STDEV_THRESHOLD].asDouble(), 255.0);
        stdevMask = selectByArea(stdevMask, 3, std::numeric_limits<int>::max());

        const double stdevDil = recipe[DILATION_STDEV].asDouble();
        if (stdevDil < 0.0)
            stdevMask = erodeCircle(stdevMask, -stdevDil);
        else
            stdevMask = dilateCircle(stdevMask, stdevDil);

        cv::Mat thMask = thresholdRange(gray, recipe[ORIGINAL_THRESHOLD].asDouble(), 255.0);
        thMask = selectByArea(thMask, 3, std::numeric_limits<int>::max());
        thMask = dilateCircle(thMask, recipe[DILATION].asDouble());

        return unionMask(stdevMask, thMask);
    }

    cv::Mat GetStdDevFilterFillup(const Json::Value& recipe, const cv::Mat& regionInput) const
    {
        const int fillMode = recipe[FILLUP_MODE].asInt();
        if (fillMode == 0)
            return fillAllHoles(regionInput);

        const int fillArea = std::max(1, recipe[FILLUP_AREA].asInt());
        return fillHolesByArea(regionInput, fillArea);
    }

    cv::Mat GetEnhanceOmitFilter(const Json::Value& recipe,
                                 const cv::Mat& imageInput,
                                 const cv::Mat& regionTotalRoi,
                                 const cv::Mat& stdevImage,
                                 const cv::Mat& stdevRegionOutput) const
    {
        cv::Mat regionOutput;
        const cv::Mat gray = ensureGray8(imageInput);
        const cv::Mat roi = ensureMask8(regionTotalRoi);

        if (recipe[WEAKSC_USE].asInt() != 0)
        {
            const double weakResize = std::max(1.0, recipe[WEAKSC_RESIZE].asDouble());
            cv::Mat reduced = reduceDomain(gray, roi);

            cv::Mat resized;
            cv::resize(reduced, resized, cv::Size(), 1.0 / weakResize, 1.0 / weakResize, cv::INTER_LINEAR);

            cv::Mat edges;
            cv::Canny(resized,
                      edges,
                      recipe[WEAKSC_LOWER_TH].asDouble(),
                      recipe[WEAKSC_UPPER_TH].asDouble());

            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP(edges, lines, 1.0, CV_PI / 180.0, 10, recipe[WEAKSC_MIN_LENGTH_SPLIT].asDouble(), 5.0);

            cv::Mat weakMask = cv::Mat::zeros(edges.size(), CV_8U);
            for (const auto& l : lines)
            {
                const double dx = static_cast<double>(l[2] - l[0]);
                const double dy = static_cast<double>(l[3] - l[1]);
                const double len = std::sqrt(dx * dx + dy * dy);
                if (len < recipe[WEAKSC_MIN_UNION_LENGTH].asDouble())
                    continue;
                cv::line(weakMask, {l[0], l[1]}, {l[2], l[3]}, cv::Scalar(255), 1, cv::LINE_8);
            }

            cv::resize(weakMask, weakMask, gray.size(), 0.0, 0.0, cv::INTER_NEAREST);
            weakMask = erodeCircle(weakMask, weakResize / 2.0 - 0.5);
            weakMask = dilateCircle(weakMask, recipe[DILATION].asDouble());

            regionOutput = unionMask(stdevRegionOutput, weakMask);
        }

        if (recipe[STICKER_USE].asInt() != 0)
        {
            cv::Mat stickerSeed = thresholdRange(ensureGray8(stdevImage), recipe[STDEV_THRESHOLD].asDouble(), 255.0);
            stickerSeed = selectByArea(stickerSeed,
                                       std::max(1, static_cast<int>(recipe[STICKER_AREA_MIN].asDouble() / 20.0)),
                                       std::numeric_limits<int>::max());
            stickerSeed = morphCircle(stickerSeed, cv::MORPH_CLOSE, recipe[STICKER_CLOSING].asDouble());

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(stickerSeed.clone(), contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

            cv::Mat sticker = cv::Mat::zeros(stickerSeed.size(), CV_8U);
            const int minHoles = recipe[STICKER_HOLES_NUM_MIN].asInt();
            const double minArea = recipe[STICKER_AREA_MIN].asDouble();
            const double maxArea = recipe[STICKER_AREA_MAX].asDouble();

            for (size_t i = 0; i < contours.size(); ++i)
            {
                if (hierarchy[i][3] >= 0)
                    continue; // only outer components

                const double area = std::abs(cv::contourArea(contours[i]));
                if (area < minArea || area > maxArea)
                    continue;

                const int holes = childHoleCount(static_cast<int>(i), hierarchy);
                if (holes < minHoles)
                    continue;

                cv::drawContours(sticker, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
            }

            sticker = fillAllHoles(sticker);
            sticker = dilateCircle(sticker, recipe[STICKER_DILATION].asDouble());

            regionOutput = unionMask(regionOutput, sticker);

            const cv::Mat selected = GetSelectedRegion(recipe, gray, roi);
            regionOutput = GetEnhanceOmitFilterConcat(regionOutput, selected);
        }

        return ensureMask8(regionOutput);
    }

    cv::Mat GetSelectedRegion(const Json::Value& recipe,
                              const cv::Mat& imageInput,
                              const cv::Mat& regionTotalRoi) const
    {
        const cv::Mat gray = ensureGray8(imageInput);
        const cv::Mat roi = ensureMask8(regionTotalRoi);
        const cv::Mat process = reduceDomain(gray, roi);

        const int filterSize = std::max(1, recipe[DYNAMIC_FILTER_SIZE].asInt()) | 1;
        const int thresholdValue = recipe[DYNAMIC_THRESHOLD].asInt();

        cv::Mat mean;
        cv::blur(process, mean, cv::Size(filterSize, filterSize));

        cv::Mat diff;
        cv::subtract(process, mean, diff, cv::noArray(), CV_16S);

        cv::Mat dynMask = cv::Mat::zeros(process.size(), CV_8U);
        for (int y = 0; y < diff.rows; ++y)
        {
            const short* dptr = diff.ptr<short>(y);
            const uchar* rptr = roi.ptr<uchar>(y);
            uchar* optr = dynMask.ptr<uchar>(y);
            for (int x = 0; x < diff.cols; ++x)
            {
                if (rptr[x] == 0)
                    continue;
                if (dptr[x] > thresholdValue)
                    optr[x] = 255;
            }
        }

        return selectByArea(dynMask, 50, 99999);
    }

    cv::Mat GetEnhanceOmitFilterConcat(const cv::Mat& regionInput, const cv::Mat& selectedRegion) const
    {
        return unionMask(regionInput, selectedRegion);
    }

    cv::Mat GetOmitImage(const cv::Mat& regionTotalRoi,
                         const cv::Mat& imageInput,
                         const cv::Mat& selectedRegion,
                         double resizeFactorVignetting) const
    {
        (void)regionTotalRoi;
        (void)imageInput;

        const cv::Mat selected = ensureMask8(selectedRegion);
        if (selected.empty())
            return {};

        const double dResizeFactor = std::round((1.0 / std::max(1e-6, resizeFactorVignetting)) * 10000.0) / 10000.0;
        cv::Mat scaled;
        cv::resize(selected, scaled, cv::Size(), dResizeFactor, dResizeFactor, cv::INTER_NEAREST);

        cv::Mat th;
        cv::threshold(scaled, th, 30, 255, cv::THRESH_BINARY);

        cv::Mat labels, stats, centroids;
        const int n = cv::connectedComponentsWithStats(th, labels, stats, centroids, 8, CV_32S);

        cv::Mat out = cv::Mat::zeros(th.size(), CV_8U);
        for (int i = 1; i < n; ++i)
        {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 10 || area > 200)
                continue;

            const cv::Rect r(stats.at<int>(i, cv::CC_STAT_LEFT),
                             stats.at<int>(i, cv::CC_STAT_TOP),
                             stats.at<int>(i, cv::CC_STAT_WIDTH),
                             stats.at<int>(i, cv::CC_STAT_HEIGHT));

            const double ratio = compRatio(r);
            if (!((ratio >= 0.0 && ratio <= 0.5) || (ratio >= 2.0 && ratio <= 15.0)))
                continue;

            out.setTo(255, labels == i);
        }

        if (cv::countNonZero(out) == 0)
            return cv::Mat::zeros(selected.rows, selected.cols, CV_8U);

        return out;
    }

    cv::Mat GetSharedSelectedRegion(const Json::Value& recipe,
                                    const cv::Mat& imageInput,
                                    const cv::Mat& regionTotalRoi) const
    {
        cv::Mat intersect;
        cv::bitwise_and(ensureMask8(imageInput), ensureMask8(regionTotalRoi), intersect);

        const int minArea = std::max(1, static_cast<int>(recipe[SHARED_BLOB_MIN_AREA].asDouble()));
        cv::Mat selected = selectByArea(intersect, minArea, std::numeric_limits<int>::max());
        selected = intersectionMask(selected, regionTotalRoi);
        return selected;
    }

    cv::Mat GetInspectionRoi(const Json::Value& recipe, const cv::Mat& srcImage) const
    {
        const cv::Mat gray = ensureGray8(srcImage);

        cv::Mat region = thresholdRange(gray,
                                        recipe[INSP_ROI_MIN_THRESHOLD].asDouble(),
                                        recipe[INSP_ROI_MAX_THRESHOLD].asDouble());

        region = morphCircle(region, cv::MORPH_OPEN, 1.5);
        region = morphCircle(region, cv::MORPH_CLOSE, 3.5);

        cv::Mat labels, stats, centroids;
        const int n = cv::connectedComponentsWithStats(region, labels, stats, centroids, 8, CV_32S);
        if (n <= 1)
            return cv::Mat::zeros(gray.size(), CV_8U);

        int bestIdx = 1;
        int bestArea = stats.at<int>(1, cv::CC_STAT_AREA);
        for (int i = 2; i < n; ++i)
        {
            const int a = stats.at<int>(i, cv::CC_STAT_AREA);
            if (a > bestArea)
            {
                bestArea = a;
                bestIdx = i;
            }
        }

        cv::Mat maxRegion = cv::Mat::zeros(region.size(), CV_8U);
        maxRegion.setTo(255, labels == bestIdx);
        maxRegion = fillAllHoles(maxRegion);

        std::vector<cv::Point> nz;
        cv::findNonZero(maxRegion, nz);
        if (nz.empty())
            return cv::Mat::zeros(gray.size(), CV_8U);

        cv::Rect bbox = cv::boundingRect(nz);
        cv::Mat out = cv::Mat::zeros(gray.size(), CV_8U);
        cv::rectangle(out, bbox, cv::Scalar(255), cv::FILLED);
        out = erodeCircle(out, 1.5);
        return out;
    }

    void CalcOmitDefectData(const cv::Mat& omitInspRoi,
                            const cv::Mat& resultRoi,
                            int& omitResultAreaCount,
                            int& omitResultAreaRate) const
    {
        const int inspArea = cv::countNonZero(ensureMask8(omitInspRoi));
        const int resultArea = cv::countNonZero(ensureMask8(resultRoi));

        omitResultAreaCount = resultArea;
        if (inspArea <= 0)
        {
            omitResultAreaRate = 0;
            return;
        }

        omitResultAreaRate = static_cast<int>((static_cast<double>(resultArea) / static_cast<double>(inspArea)) * 10000.0);
    }
};
