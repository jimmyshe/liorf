//
// Created by jimmy on 23-9-13.
//

#ifndef LIORF_KISSODOMETRY_H
#define LIORF_KISSODOMETRY_H

#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"

class KissOdometry {

public:

private:
    kiss_icp::pipeline::KISSConfig config_;
    std::unique_ptr<kiss_icp::AdaptiveThreshold> adaptive_threshold_;
};


#endif//LIORF_KISSODOMETRY_H
