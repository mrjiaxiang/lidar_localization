#include "lidar_localization/front/LK/feature_extractor.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include <glog/logging.h>

using namespace cv;
using namespace std;

namespace lidar_localization {

const int EDGE_THRESHOLD = 19;
const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;

void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2,
                               ExtractorNode &n3, ExtractorNode &n4) {
    const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
    const int halfY = ceil(static_cast<float>(UR.y - UL.y) / 2);

    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x + halfX, UL.y);
    n1.BL = cv::Point2i(UL.x, UL.y + halfY);
    n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x, UL.y + halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x, BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    for (size_t i = 0; i < vKeys.size(); i++) {
        const cv::KeyPoint &kp = vKeys[i];
        if (kp.pt.x < n1.UR.x) {
            if (kp.pt.y < n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        } else if (kp.pt.y < n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }
    if (n1.vKeys.size() == 1)
        n1.bNoMore = true;
    if (n2.vKeys.size() == 1)
        n2.bNoMore = true;
    if (n3.vKeys.size() == 1)
        n3.bNoMore = true;
    if (n4.vKeys.size() == 1)
        n4.bNoMore = true;
}

static bool compareNodes(pair<int, ExtractorNode *> &e1,
                         pair<int, ExtractorNode *> &e2) {
    if (e1.first < e2.first) {
        return true;
    } else if (e1.first > e2.first) {
        return false;
    } else {
        if (e1.second->UL.x < e2.second->UL.x) {
            return true;
        } else {
            return false;
        }
    }
}

FeatureExtractor::FeatureExtractor() {}

FeatureExtractor::FeatureExtractor(int nfeatures, float scaleFactor,
                                   int nlevels)
    : nfeatures_(nfeatures), scaleFactor_(scaleFactor), nlevels_(nlevels) {

    mvScaleFactor_.resize(nlevels_);
    mvInvLevelSigma2_.resize(nlevels_);
    mvScaleFactor_[0] = 1.0f;
    mvLevelSigma2_[0] = 1.0f;

    for (int i = 0; i < nlevels_; i++) {
        mvScaleFactor_[i] = mvScaleFactor_[i - 1] * scaleFactor_;
        mvLevelSigma2_[i] = mvScaleFactor_[i] * mvScaleFactor_[i];
    }

    mvInvScaleFactor_.resize(nlevels_);
    mvInvLevelSigma2_.resize(nlevels_);
    for (int i = 0; i < nlevels_; i++) {
        mvInvScaleFactor_[i] = 1.0f / mvScaleFactor_[i];
        mvInvLevelSigma2_[i] = 1.0f / mvLevelSigma2_[i];
    }

    mvImagePyramid_.resize(nlevels_);

    mnFeaturesPerLevel_.resize(nlevels_);

    // 将特征点分布到每一层
    float factor = 1.0f / scaleFactor_;
    float nDesiredFeaturesPerScale =
        nfeatures_ * (1 - factor) /
        (1 - (float)pow((double)factor, (double)nlevels_));

    int sumFeatures = 0;
    for (int level = 0; level < nlevels_ - 1; level++) {
        mnFeaturesPerLevel_[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel_[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel_[nlevels_ - 1] = std::max(nfeatures - sumFeatures, 0);
}

FeatureExtractor::~FeatureExtractor() {}

void FeatureExtractor::ComputePyramid(cv::Mat image) {
    for (int level = 0; level < nlevels_; ++level) {
        float scale = mvInvScaleFactor_[level];
        Size sz(cvRound((float)image.cols * scale),
                cvRound((float)image.rows * scale));

        Size whole_size(sz.width + EDGE_THRESHOLD * 2,
                        sz.height + EDGE_THRESHOLD * 2);

        Mat temp(whole_size, image.type()), masktemp;

        mvImagePyramid_[level] =
            temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        if (level != 0) {
            resize(mvImagePyramid_[level - 1], mvImagePyramid_[level], sz, 0, 0,
                   INTER_LINEAR);
            copyMakeBorder(mvImagePyramid_[level], temp, EDGE_THRESHOLD,
                           EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101 + BORDER_ISOLATED);
        } else {
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           EDGE_THRESHOLD, EDGE_THRESHOLD, BORDER_REFLECT_101);
        }
    }
}

void FeatureExtractor::ComputeKeyPointsOctTree(
    std::vector<std::vector<cv::KeyPoint>> &allKeypoints) {

    allKeypoints.resize(nlevels_);
    const float W = 35;

    for (int level = 0; level < nlevels_; ++level) {
        const int minBorderX = EDGE_THRESHOLD - 3;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid_[level].cols - EDGE_THRESHOLD + 3;
        const int maxBorderY = mvImagePyramid_[level].rows - EDGE_THRESHOLD + 3;

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures_ * 10);

        const float width = (maxBorderX - minBorderX);
        const float height = (maxBorderY - minBorderY);

        const int nCols = width / W;
        const int nRows = height / W;
        const int wCell = ceil(width / nCols);
        const int hCell = ceil(height / nRows);

        for (int i = 0; i < nRows; i++) {
            const float iniY = minBorderY + i * hCell;
            float maxY = iniY + hCell + 6;

            if (iniY >= maxBorderY - 3)
                continue;
            if (maxY > maxBorderY)
                maxY = maxBorderY;

            for (int j = 0; j < nCols; j++) {
                const float iniX = minBorderX + j * wCell;
                float maxX = iniX + wCell + 6;
                if (iniX >= maxBorderX - 6)
                    continue;
                if (maxX > maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;

                FAST(mvImagePyramid_[level]
                         .rowRange(iniY, maxY)
                         .colRange(iniX, maxX),
                     vKeysCell, iniThFAST_, true);

                if (vKeysCell.empty()) {
                    FAST(mvImagePyramid_[level]
                             .rowRange(iniY, maxY)
                             .colRange(iniX, maxX),
                         vKeysCell, minThFAST_, true);
                }

                if (!vKeysCell.empty()) {
                    for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin();
                         vit != vKeysCell.end(); vit++) {
                        (*vit).pt.x += j * wCell;
                        (*vit).pt.y += i * hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }
            }
        }

        vector<KeyPoint> &keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures_);

        keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                      minBorderY, maxBorderY,
                                      mnFeaturesPerLevel_[level], level);

        const int scaledPatchSize = PATCH_SIZE * mvScaleFactor_[level];

        // Add border to coordinates and scale information
        const int nkps = keypoints.size();
        for (int i = 0; i < nkps; i++) {
            keypoints[i].pt.x += minBorderX;
            keypoints[i].pt.y += minBorderY;
            keypoints[i].octave = level;
            keypoints[i].size = scaledPatchSize;
        }
    }
}

int FeatureExtractor::operator()(cv::InputArray _image, cv::InputArray _mask,
                                 std::vector<cv::KeyPoint> &_keypoints) {

    if (_image.empty()) {
        return -1;
    }
    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1);

    ComputePyramid(image);

    vector<vector<KeyPoint>> allKeypoints;
    ComputeKeyPointsOctTree(allKeypoints);
    int nkeypoints = 0;
    for (int level = 0; level < nlevels_; ++level)
        nkeypoints += (int)allKeypoints[level].size();

    LOG(INFO) << "nkeypoints = " << nkeypoints;
}

std::vector<cv::KeyPoint> FeatureExtractor::DistributeOctTree(
    const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
    const int &maxX, const int &minY, const int &maxY, const int &N,
    const int &level) {

    // Compute how many initial nodes
    const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

    const float hX = static_cast<float>(maxX - minX) / nIni;

    list<ExtractorNode> lNodes;

    vector<ExtractorNode *> vpIniNodes;
    vpIniNodes.resize(nIni);

    for (int i = 0; i < nIni; i++) {
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
        ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
        ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
        ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    // Associate points to childs
    for (size_t i = 0; i < vToDistributeKeys.size(); i++) {
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
    }

    list<ExtractorNode>::iterator lit = lNodes.begin();

    while (lit != lNodes.end()) {
        if (lit->vKeys.size() == 1) {
            lit->bNoMore = true;
            lit++;
        } else if (lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;

    int iteration = 0;

    vector<pair<int, ExtractorNode *>> vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size() * 4);

    while (!bFinish) {
        iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while (lit != lNodes.end()) {
            if (lit->bNoMore) {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            } else {
                // If more than one point, subdivide
                ExtractorNode n1, n2, n3, n4;
                lit->DivideNode(n1, n2, n3, n4);

                // Add childs if they contain points
                if (n1.vKeys.size() > 0) {
                    lNodes.push_front(n1);
                    if (n1.vKeys.size() > 1) {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n1.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (n2.vKeys.size() > 0) {
                    lNodes.push_front(n2);
                    if (n2.vKeys.size() > 1) {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n2.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (n3.vKeys.size() > 0) {
                    lNodes.push_front(n3);
                    if (n3.vKeys.size() > 1) {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n3.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if (n4.vKeys.size() > 0) {
                    lNodes.push_front(n4);
                    if (n4.vKeys.size() > 1) {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(n4.vKeys.size(), &lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit = lNodes.erase(lit);
                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize) {
            bFinish = true;
        } else if (((int)lNodes.size() + nToExpand * 3) > N) {

            while (!bFinish) {

                prevSize = lNodes.size();

                vector<pair<int, ExtractorNode *>> vPrevSizeAndPointerToNode =
                    vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),
                     vPrevSizeAndPointerToNode.end(), compareNodes);
                for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0;
                     j--) {
                    ExtractorNode n1, n2, n3, n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3,
                                                                    n4);

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0) {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1) {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0) {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1) {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0) {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1) {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0) {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1) {
                            vSizeAndPointerToNode.push_back(
                                make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if ((int)lNodes.size() >= N)
                        break;
                }

                if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
                    bFinish = true;
            }
        }
    }

    // Retain the best point in each node
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nfeatures_);
    for (list<ExtractorNode>::iterator lit = lNodes.begin();
         lit != lNodes.end(); lit++) {
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        cv::KeyPoint *pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for (size_t k = 1; k < vNodeKeys.size(); k++) {
            if (vNodeKeys[k].response > maxResponse) {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

} // namespace lidar_localization