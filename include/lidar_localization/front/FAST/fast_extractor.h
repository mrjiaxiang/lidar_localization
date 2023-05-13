#pragma once

#include <list>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace lidar_localization {

class ExtractorNode {
  public:
    ExtractorNode() : bNoMore(false) {}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3,
                    ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class FeatureExtractor {
  public:
    FeatureExtractor();
    FeatureExtractor(int nfeatures_, float scaleFactor, int nlevels,
                     int initThFast, int minThFast);

    ~FeatureExtractor();

    int operator()(cv::InputArray _image, cv::InputArray _mask,
                   std::vector<cv::KeyPoint> &_keypoints);

    std::vector<cv::KeyPoint>
    DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys,
                      const int &minX, const int &maxX, const int &minY,
                      const int &maxY, const int &N, const int &level);

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(
        std::vector<std::vector<cv::KeyPoint>> &allKeypoints);

        int inline GetLevels() { return nlevels_; }

    float inline GetScaleFactor() { return scaleFactor_; }

    std::vector<float> inline GetScaleFactors() { return mvScaleFactor_; }

    std::vector<float> inline GetInverseScaleFactors() {
        return mvInvScaleFactor_;
    }

    std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2_; }

    std::vector<float> inline GetInverseScaleSigmaSquares() {
        return mvInvLevelSigma2_;
    }

  public:
    std::vector<cv::Mat> mvImagePyramid_;
    std::vector<int> mnFeaturesPerLevel_;
    std::vector<int> umax_;

  private:
    int nfeatures_;

    int nlevels_;
    double scaleFactor_;
    int iniThFAST;
    int minThFAST;
    std::vector<float> mvScaleFactor_;
    std::vector<float> mvInvScaleFactor_;
    std::vector<float> mvLevelSigma2_;
    std::vector<float> mvInvLevelSigma2_;

    int iniThFAST_;
    int minThFAST_;

    cv::Ptr<cv::FastFeatureDetector> fast_detector_;
    cv::Ptr<cv::GFTTDetector> gftt_detector_;
};

} // namespace lidar_localization