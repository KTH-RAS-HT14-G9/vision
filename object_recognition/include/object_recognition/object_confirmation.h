#ifndef OBJECT_CONFIRMATION_H
#define OBJECT_CONFIRMATION_H

#include <common/parameter.h>
#include <common/object_classification.h>

static const int PHASE_DETECTION = 0;
static const int PHASE_RECOGNITION = 1;

class ObjectConfirmation
{
public:
    ObjectConfirmation();

    bool update(const common::ObjectClassification& classification, common::ObjectClassification& confirmed_object, int phase);

protected:

    typedef std::pair<std::string, std::string> Case;

    Case correct_classification(const std::string &name_color, const std::string &name_shape);
    void increment(std::map<std::string, int>& map, const common::Classification& name);
    void update_shape_attribute(const common::ObjectClassification& classification);
    double calculate_max_ratio(const std::map<std::string, int>& map, std::string& max_key);
    void reset_accumulation();
    void set_special_cases_map();

    Parameter<double> _min_iterations_p1;
    Parameter<double> _min_ratio_p1;
    Parameter<int> _reset_threshold_p1;

    Parameter<double> _min_iterations_p2;
    Parameter<double> _min_ratio_p2;
    Parameter<int> _reset_threshold_p2;

    std::map<std::string, common::ObjectClassification> _last_attributes;
    std::map<std::string, int> _shape_accumulator;
    std::map<std::string, int> _color_accumulator;

    std::map<Case, Case> _special_case_map;

    int _empty_frames;
    int _accumulated_frames;
    int _last_phase;
};

#endif // OBJECT_CONFIRMATION_H
