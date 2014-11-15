#ifndef COLOR_CLASSIFIER_H
#define COLOR_CLASSIFIER_H

#define NBINS 360

class ColorClassifier
{
public:
    ColorClassifier(double reference_hue);

    double classify(const std::vector<double>& hues);

protected:
    double _reference_hue;
    std::vector<double> _histogram;
    double weight_color(double hue);
    void build_histogram(double hue);

};

ColorClassifier::ColorClassifier(double reference_hue)
    :_reference_hue(reference_hue)
{
    _histogram.resize(NBINS);
}

double ColorClassifier::weight_color(double hue)
{   // this doesn't really help to solve the red and orange problem
    int window_size=100;
    if(std::abs(_reference_hue- hue)<(window_size/2)) return (1-(std::abs(_reference_hue-hue)/(window_size/2)));
    else if (std::abs(_reference_hue-(hue+NBINS))<(window_size/2))
    {
        return (1-(std::abs(_reference_hue-(hue+NBINS))/(window_size/2)));
    }
    else if (std::abs(_reference_hue-(hue-NBINS))<(window_size/2))
    {
        return (1-(std::abs(_reference_hue-(hue-NBINS))/(window_size/2)));
    }
    else return 0;

   /*
   // return std::abs(hue-_reference_hue)/(NBINS/2);

  // weight considering it's this hue
  // hypothesis the hue values wrapp up around 255, meaning 0 and 255 are the same hue
   if(std::abs(_reference_hue-hue)<(NBINS/2)) return (1-(std::abs(_reference_hue-hue)/(NBINS/2)));
   else if (hue >_reference_hue)
   {
       double h_temp=hue-(NBINS-1);
       return (1-(std::abs(_reference_hue-h_temp)/(NBINS/2)));

    }
   else
   {
      double h_temp=hue+NBINS-1;
       return (1-(std::abs(_reference_hue-h_temp)/(NBINS/2)));
    }
    */

}

void ColorClassifier::build_histogram(double hue)
{
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
        _histogram[idx]=_histogram[idx]+gauss_mask[i+2]* weight_color(hue);
        //std::cout << _reference_hue << "\t" << hue << "\t" << weight_color(hue)<<"\n";
    }
}


double ColorClassifier::classify(const std::vector<double> &hues)
{
    double probability = 0.0;
    _reference_hue=round(_reference_hue);

    // add hue values to the histogram with weight;
    //std::cout<< hues.size()<<std::endl;

    for (int i=0;i< hues.size();i++ )
    {
     build_histogram(hues[i]);
    }

    // find the peak vaule around the reference hue
    int max=0;
    int neighbour=15;
    for ( int i=(-(neighbour-1)/2); i<(neighbour-(neighbour-1)/2);i++)
    {
        if (_histogram[_reference_hue+i] > max) max=_histogram[_reference_hue+i];
    }

    double sum=0;
    for (int i=0;i<hues.size();++i)
    {
        sum=sum+_histogram[i];
    }

    probability=(double)max/sum;
    // since we only give weights to the area close to the reference hue, meaning sometimes the histogram only have 0;
    if (sum==0) probability=0;
    //std::cout<< _reference_hue << "\t" <<sum <<std::endl;
    return probability;
}

#endif // COLOR_CLASSIFIER_H
