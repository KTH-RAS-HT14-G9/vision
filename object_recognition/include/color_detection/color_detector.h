#ifndef COLOR_DETECTOR_H
#define COLOR_DETECTOR_H

#include <common/types.h>
#include "color_classifier.h"

class ColorDetector
{
public:
    ColorDetector();

    common::NameAndProbability classify(const common::PointCloudRGB::Ptr &roi);

protected:

    double extractHue(int rgb);

    std::vector<ColorClassifier> _classifiers;
    std::vector<double> _hues;
};

#endif // COLOR_DETECTOR_H
