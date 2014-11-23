#include "object_recognition/object_confirmation.h"

ObjectConfirmation::ObjectConfirmation()
    :_min_ratio("/vision/recognition/confirmation/min_ratio",0.6)
    ,_min_iterations("/vision/recognition/confirmation/min_frames",5)
    ,_reset_threshold("/vision/recognition/confirmation/reset_threshold",10)
    ,_empty_frames(0)
    ,_accumulated_frames(0)
{
}

void ObjectConfirmation::increment(std::map<std::string,int> &map, const common::NameAndProbability &name)
{
    std::map<std::string,int>::iterator it = map.find(name.name());
    if( it == map.end() ) {
        std::pair<std::string,int> pair;
        pair.first = name.name();
        pair.second = 1;
        map.insert(pair);
    }
    else {
        it->second++;
    }
}

double ObjectConfirmation::calculate_max_ratio(const std::map<std::string,int> &map, std::string &max_key)
{
    int max = 0;
    int sum = 0;
    for(std::map<std::string,int>::const_iterator it = map.begin(); it != map.end(); ++it)
    {
        if (it->second > max) {
            max = it->second;
            max_key = it->first;
        }

        sum += it->second;
    }

    sum -= max;

    return (double)max/(double)sum;
}

void ObjectConfirmation::reset_accumulation()
{
	ROS_INFO("-------------- RESET --------------");
	_accumulated_frames = 0;
    _shape_accumulator.clear();
    _color_accumulator.clear();
}

bool ObjectConfirmation::update(const common::NameAndProbability &shape,
                                const common::NameAndProbability &color,
                                std::string &confirmed_object)
{
    if (shape.is_undefined()) {
        _empty_frames++;

        if (_empty_frames >= _reset_threshold()) {
            reset_accumulation();
			_empty_frames = 0;
        }

        return false;
    }

    _accumulated_frames++;

    if (!shape.is_undefined()) increment(_shape_accumulator,shape);
    if (!color.is_undefined()) increment(_color_accumulator,color);


    //--------------------------------------------------------------------------
    // Evaluation

    if (_accumulated_frames > _min_iterations())
    {
        std::string name_shape;
        std::string name_color;

        double ratio_shape = calculate_max_ratio(_shape_accumulator,name_shape);
        double ratio_color = calculate_max_ratio(_color_accumulator,name_color);

        if (ratio_color > _min_ratio() && name_color.compare("plurple") == 0)
        {
            confirmed_object = "purple cross";
            reset_accumulation();
            return true;
        }

        confirmed_object = "";
        if (ratio_color > _min_ratio())
        {
            confirmed_object = name_color;
        }
        if (ratio_shape > _min_ratio())
        {
            confirmed_object += " " + name_shape;
        }

        if (confirmed_object.empty())
            confirmed_object = "object";


        reset_accumulation();
        return true;

    }


    return false;
}
