#include "object_recognition/object_confirmation.h"

ObjectConfirmation::ObjectConfirmation()
    :_min_ratio("/vision/recognition/confirmation/min_ratio",0.6)
    ,_min_iterations("/vision/recognition/confirmation/min_frames",5)
    ,_reset_threshold("/vision/recognition/confirmation/reset_threshold",5)
    ,_empty_frames(0)
    ,_accumulated_frames(0)
{
}

void ObjectConfirmation::increment(std::map<std::string,int> &map, const common::Classification &name)
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

void ObjectConfirmation::update_shape_attribute(const common::ObjectClassification& classification)
{
    std::pair<std::string,common::ObjectClassification> pair;
    pair.first = classification.shape().name();
    pair.second = classification;

    if (_last_attributes.find(pair.first) != _last_attributes.end())
        _last_attributes.erase(pair.first);

    _last_attributes.insert(pair);
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
	//ROS_ERROR("-------------- RESET --------------");
	_accumulated_frames = 0;
    _shape_accumulator.clear();
    _color_accumulator.clear();
}

bool ObjectConfirmation::update(const common::ObjectClassification& classification,
                                common::ObjectClassification &confirmed_object)
{
    //if shape is undefined and the color is not plurple
    if (classification.shape().is_undefined() && classification.color().name().compare("plurple") != 0) {
        //should we add orange and light_green as well

//	ROS_ERROR("Shape: %s, Color: %s",classification.shape().name().c_str(), classification.color().name().c_str());
        _empty_frames++;

        if (_empty_frames >= _reset_threshold()) {
            reset_accumulation();
			_empty_frames = 0;
        }

        return false;
    }

    _empty_frames = 0;
    _accumulated_frames++;

    if (!classification.shape().is_undefined()) {
        increment(_shape_accumulator,classification.shape());
        update_shape_attribute(classification);
    }
    if (!classification.color().is_undefined()) {
        increment(_color_accumulator,classification.color());
    }


    //--------------------------------------------------------------------------
    // Evaluation

    //ROS_ERROR("Accumulated frames: %d", _accumulated_frames);
    if (_accumulated_frames > _min_iterations())
    {
        std::string name_shape;
        std::string name_color;

        double ratio_shape = calculate_max_ratio(_shape_accumulator,name_shape);
        double ratio_color = calculate_max_ratio(_color_accumulator,name_color);

        if (ratio_color > _min_ratio() && name_color.compare("plurple") == 0)
            // consistently detected purple
        {
            confirmed_object = common::ObjectClassification(
                        common::Classification("cross",1), //if purple then purple cross
                        common::Classification(name_color,1));
            reset_accumulation();
            return true;
        }
        if (ratio_color > _min_ratio() && name_color.compare("green_light") == 0)
            // added with same logic as purple - maybe unecessary because it usualy finds correct shape and color
        {
            confirmed_object = common::ObjectClassification(
                        common::Classification("cylinder",1), //if green_light then cylinder
                        common::Classification("green",1)); // when color is green_light, change name to green
            reset_accumulation();
            return true;
        }
        if (ratio_color > _min_ratio() && name_color.compare("orange") == 0)
            // added with samwe logic as purple
        {
            confirmed_object = common::ObjectClassification(
                        common::Classification("patric",1), //if orange then patric
                        common::Classification(name_color,1));
            reset_accumulation();
            return true;
        }
        if (ratio_shape > _min_ratio() && name_shape.compare("cylinder") == 0 && ratio_color > _min_ratio() && name_color.compare("blue") == 0)
            // same logic as above but to detect the blue triangle
        {
            confirmed_object = common::ObjectClassification(
                        common::Classification("triangle",1), //if blue and cylinder then blue triangle
                        common::Classification(name_color,1));
            reset_accumulation();
            return true;
        }

        common::Classification color,shape;
        if (ratio_color > _min_ratio())
        {
            if (name_color.compare("green_light")==0)
                // when color is green_light, change name to green
                name_color="green";

            color = common::Classification(name_color,1);
        }
        if (ratio_shape > _min_ratio())
        {
            shape = _last_attributes.at(name_shape).shape();
        }

        confirmed_object = common::ObjectClassification(shape,color);

        reset_accumulation();
        return true;

    }


    return false;
}
