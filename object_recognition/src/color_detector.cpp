#include "color_detection/color_detector.h"
#include <pcl/point_types_conversion.h>

ColorDetector::ColorDetector()
{   
    // initialize reference hues classifers
    common::ObjectColorMap& colormap = common::ObjectColorMap::instance();
    const std::vector<std::string>& color_names = colormap.names();
    for (int i = 0; i < color_names.size() ; ++i)
    {
        _classifiers.push_back(ColorClassifier(color_names[i],colormap.get(color_names[i])));
    }
}

double ColorDetector::extractHue(int rgb)
{
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8)  & 0x0000ff;
    uint8_t b = (rgb)       & 0x0000ff;

    float h,s,v;

    const unsigned char max = std::max (r, std::max (g, b));
    const unsigned char min = std::min (r, std::min (g, b));

    v = static_cast <float> (max) / 255.f;

    if (max == 0) // division by zero
    {
      return 0;
    }

    const float diff = static_cast <float> (max - min);
    s = diff / static_cast <float> (max);

    if (min == max) // diff == 0 -> division by zero
    {
      return 0;
    }

    if      (max == r) h = 60.f * (      static_cast <float> (g - b) / diff);
    else if (max == g) h = 60.f * (2.f + static_cast <float> (b - r) / diff);
    else               h = 60.f * (4.f + static_cast <float> (r - g) / diff); // max == b

    if (h < 0.f) h += 360.f;

    return h;
}

common::Classification ColorDetector::classify(const common::PointCloudRGB::Ptr &roi)
{
    //--------------------------------------------------------------------------
    // Convert
    _hues.clear();

    int numpoints = roi->size();

    if(numpoints==0) return common::Classification();

    _hues.resize(numpoints);
    for(int j=0; j < numpoints; j++)
    {
        const pcl::PointXYZRGB& p = roi->at(j);
        _hues[j] = extractHue(*reinterpret_cast<const int*>(&p.rgb));
    }


    //--------------------------------------------------------------------------
    // Classify

    double max_probability = 0;
    int best_i = -1;
    double green_probability =0;
    int green_index=0;
    for (int i = 0; i < _classifiers.size(); ++i)
    {
        ColorClassifier& classifier = _classifiers[i];
        double probability = classifier.classify(_hues);
        std::cout << probability << std::endl;
        if (probability > max_probability)
        {
            max_probability = probability;
            best_i = i;
        }
        if (_classifiers[i].name().compare("green") == 0) {
            green_probability = probability;
            green_index=i;
        }
    }

    if (green_probability > 0.4) {
        best_i = green_index;
        max_probability = green_probability;
    }

    if (best_i==-1)
    {
        std::cout << "It is uncolored" << " with probability " << max_probability<< std::endl;
        _classifiers.push_back(ColorClassifier("uncolored",1.0));

        return common::Classification(_classifiers[best_i].name(), 1.0);
    }
    else
    {
        std::cout << "It is " << _classifiers[best_i].name()<< " with probability " << max_probability << std::endl;
        return common::Classification(_classifiers[best_i].name(), max_probability);
    }
}

