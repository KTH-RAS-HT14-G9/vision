#include "object_recognition/object_confirmation.h"

ObjectConfirmation::ObjectConfirmation()
    :_min_ratio("/vision/recognition/confirmation/min_ratio",0.6)
    ,_min_iterations("/vision/recognition/confirmation/min_frames",5)
    ,_reset_threshold("/vision/recognition/confirmation/reset_threshold",5)
    ,_empty_frames(0)
    ,_accumulated_frames(0)
{
    set_special_cases_map();
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


void ObjectConfirmation::set_special_cases_map()
//Initialize the special cases map
{
    int nCases = 6; //remember!!

    std::string special_colors[] = {"purple",  "orange",   "blue",     "blue",     "green_light",  "*"      };
    std::string special_shapes[] = {"*",       "undefined","undefined","Cylinder", "*",            "Sphere"};

    std::string real_colors[]    = {"purple",  "",         "blue",     "blue",     "green",        "*"      };
    std::string real_shapes[]    = {"Cross",   "Patrick",  "Triangle", "Triangle", "*",            "Ball"  };

    for(int i=0;i < nCases;++i)
    {
        Case special_pair = std::make_pair(special_colors[i], special_shapes[i]);
        Case real_pair = std::make_pair(real_colors[i], real_shapes[i]);

        _special_case_map.insert(std::make_pair(special_pair,real_pair));

        std::cout << "case: " << special_pair.first << " "<< special_pair.second <<" with "<<real_pair.first << " "<<real_pair.second<<std::endl;
    }

}

ObjectConfirmation::Case ObjectConfirmation::correct_classification(const std::string& name_color, const std::string& name_shape)
{
    Case pair = std::make_pair(name_color,name_shape);

    if(name_shape.compare("Sphere") == 0)
    {
        pair.first = "*";
    }
    if(name_color.compare("green_light")==0 || name_color.compare("purple")==0)
    {
        pair.second  = "*";
    }

    std::map<Case,Case>::const_iterator iterator = _special_case_map.find(pair);

    ROS_ERROR("Current: %s %s", name_color.c_str(), name_shape.c_str());

    if (iterator!=_special_case_map.end())
    // If the detected color and shape form a special case, assign real case
    {

        std::string map_color = iterator->second.first;
        std::string map_shape = iterator->second.second;

        if (map_color.compare("*") == 0) map_color = name_color;

        if (map_shape.compare("*") == 0) map_shape = name_shape;

        std::cout << "special case detected" << std::endl;
        std::cout << "replacing " << name_color << " " << name_shape << " with " << map_color <<" " << map_shape << std::endl;

        ROS_ERROR("Fixed: %s %s", map_color.c_str(), map_shape.c_str());

        return std::make_pair(map_color, map_shape);

    }

    return std::make_pair(name_color, name_shape);
}

bool isUndefined(const std::string& str) {
    return common::Classification(str,0).is_undefined();
}

bool ObjectConfirmation::update(const common::ObjectClassification& classification,
                                common::ObjectClassification &confirmed_object)
{

    Case corrected = correct_classification(classification.color().name(), classification.shape().name());
    common::Classification shape_class(corrected.second, classification.shape().probability());
    common::Classification color_class(corrected.first, classification.color().probability());

    //if shape is undefined
    if (shape_class.is_undefined()) {

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

    if (!shape_class.is_undefined()) {
        increment(_shape_accumulator,shape_class);
        update_shape_attribute(common::ObjectClassification(shape_class, color_class));
    }
    if (!color_class.is_undefined()) {
        increment(_color_accumulator,color_class);
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

        common::Classification color,shape;
        if (ratio_color > _min_ratio())
        {
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
