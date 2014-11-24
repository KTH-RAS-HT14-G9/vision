#ifndef COLOR_CLASSIFIER_H
#define COLOR_CLASSIFIER_H

#define NBINS 360
#define WINDOW_SIZE 100

class ColorClassifier
{
public:
    ColorClassifier(const std::string& name, double reference_hue);

    double classify(const std::vector<double>& hues);
    const std::string& name();

protected:
    std::string _name;

    double _reference_hue;
    int _count_ref_hue;
    int _count_orange;
    std::vector<double> _histogram;


    double weight_color(double hue);
    void build_histogram(double hue);
    void counters(double hue);
};

ColorClassifier::ColorClassifier(const std::string& name, double reference_hue)
    :_name(name),
    _reference_hue(reference_hue)
{
    _histogram.resize(NBINS);
    _count_ref_hue=0;
    _count_orange=0;
}

const std::string& ColorClassifier::name()
{
    return _name;
}

void ColorClassifier::counters(double hue){

    if (std::abs(_reference_hue - hue) < 10) _count_ref_hue++;
    else {
        if (std::abs(_reference_hue-(hue+NBINS)) < 10) _count_ref_hue++;
        if (std::abs(_reference_hue-(hue-NBINS)) < 10) _count_ref_hue++;
    }


    if (std::abs(20 - hue) < 10) _count_orange++;
    else {
        if (std::abs(20-(hue+NBINS)) < 10) _count_orange++;
        if (std::abs(20-(hue-NBINS)) < 10) _count_orange++;
    }

}

double ColorClassifier::weight_color(double hue)
{

    if(std::abs(_reference_hue- hue)<(WINDOW_SIZE/2)) return (1-(std::abs(_reference_hue-hue)/(WINDOW_SIZE/2)));
    else if (std::abs(_reference_hue-(hue+NBINS))<(WINDOW_SIZE/2))
    {
        return (1-(std::abs(_reference_hue-(hue+NBINS))/(WINDOW_SIZE/2)));
    }
    else if (std::abs(_reference_hue-(hue-NBINS))<(WINDOW_SIZE/2))
    {
        return (1-(std::abs(_reference_hue-(hue-NBINS))/(WINDOW_SIZE/2)));
    }
    else return 0;

}

void ColorClassifier::build_histogram(double hue)
{
    counters(hue);

    // _histogram[round(hue)]=_histogram[round(hue)] + weight_color(hue);
    // for each hue, spread numbers around its -2----2 neighbours
    //static const double gauss_mask[5]={0.1,0.2,0.4,0.2,0.1};
    static const double gauss_mask[5]={0,0,1,0,0};
    for (int i=-2;i<3;++i)
    {
        int idx = (round(hue)+i);

        if (idx < 0)
        {
            idx = NBINS + idx;
        }
        else if (idx >= NBINS)
        {
            idx = idx%NBINS;
        }
        if (idx < 0 || idx > _histogram.size())
            ROS_ERROR("Index out of bounds. Fix this: %d",idx);

        _histogram[idx]=_histogram[idx]+gauss_mask[i+2]* weight_color(hue);
        //std::cout << _reference_hue << "\t" << hue << "\t" << weight_color(hue)<<"\n";
    }
}




double ColorClassifier::classify(const std::vector<double> &hues)
{
    double probability = 0.0;
    _reference_hue=round(_reference_hue);

    for ( int i=0; i < NBINS;++i) _histogram[i]=0;

    // add hue values to the histogram with weight;
    //std::cout<< hues.size()<<std::endl;

    for (int i=0;i< hues.size();i++ )
    {
        build_histogram(hues[i]);
    }

    // find the peak vaĺue around the reference hue
    int max=0;
    int neighbour=7;
    for ( int i=(-(neighbour-1)/2); i<(neighbour-(neighbour-1)/2);i++)
    {
        int k = ((int)_reference_hue + i) % NBINS;

        if (_histogram[k] > max) max = _histogram[k];
    }

    double sum=0;
    for (int i=0;i < NBINS;++i)
    {
        sum=sum+_histogram[i];
    }

    probability=(double)max/sum;
    // since we only give weights to the area close to the reference hue, meaning sometimes the histogram only have 0;
    if (sum==0) probability=0;
    //std::cout<< _reference_hue << "\t" <<sum <<std::endl;

    //std::cout << _count_ref_hue << "\t" << _count_orange << "\t" << hues.size() << std::endl;

    probability=probability*_count_ref_hue/hues.size();

    /////////////

    double prob=0;
    if(_count_orange>10) prob=(double)_count_ref_hue/((double)_count_ref_hue+(double)_count_orange);
    else prob=prob=(double)_count_ref_hue/(double) hues.size();

    //std::cout << probability << "\t" << prob << std::endl;

    /////////////

    _count_ref_hue=0;
    _count_orange=0;

    return prob;
}

#endif // COLOR_CLASSIFIER_H
