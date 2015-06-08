#ifndef COMMONFUNC_H
#define COMMONFUNC_H
#include "commonHeader.h"

namespace commonFunc
{
    // overload func to calculate l2 norm
    template<typename T>
    inline float l2norm(T x, T y, T z)
    {   return std::sqrt(x*x + y*y +z*z);   }

    template<typename T>
    inline float l2norm(T x, T y)
    {
        return std::norm( std::complex<T>(x, y) );
    }

    inline float l2norm(std::vector<float> vec)
    {
        int i= vec.size();
        double sum = 0.0;
        while(i--)
        {   sum += vec.at(i)*vec.at(i);    }
        return std::sqrt(sum);
    }

    // func median value
    template<typename T>
    T getMedian(const std::vector<T> &v);

    // func mean value
    template<typename T>
    T getMean(const std::vector<T> &v);

    // func minimum value
    template <typename T>
    inline T getMinimum(const std::vector<T> &v)
    {return *std::min_element( v.begin(), v.end());}

    // func max value
    template <typename T>
    inline T getMaximum(const std::vector<T> &v)
    {return *std::min_element( v.begin(), v.end() );}

    // func to generat radom sample indices of ransac

    inline void randomIdx(u16 randIdx[], u16 idxRange)
    {
        bool stop = false;

        /* initialize random seed: */
        std::srand (std::time(NULL));

        /* generate random number from 0 to idxRange */
        s16 randNum = std::rand() % idxRange;
        randIdx[0] = (u16) randNum;
        while(!stop)
        {
            std::srand (std::time(NULL));
            randNum = std::rand() % idxRange;
            if(randIdx[0] != (u16) randNum)
            {
                randIdx[1] = (u16) randNum;
                stop = true;
            }
        }
        while(stop)
        {
            std::srand (std::time(NULL));
            randNum = std::rand() % idxRange;
            if(randIdx[0] != (u16) randNum && randIdx[1] != (u16) randNum)
            {
                randIdx[2] = (u16) randNum;
                stop = false;
            }
        }
    }
}

///////////////////////////////////
/// Median Value Estimation
///////////////////////////////////
// Calculate the median value of matching pair distance;
template<typename T>
T getMedian(const std::vector<T> &v)
{
    T median;
    std::vector<T> scores;
    for(int i=0;i<v.size();i++)
    {
        scores.push_back(v[i]);
    }
    size_t size = scores.size();

    std::sort(scores.begin(), scores.end());

    if (size  % 2 == 0)
    {
        median = (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else
    {
        median = scores[size / 2];
    }
    return median;
}
template<typename T>
// calculate mean value of vector
T getMean(const std::vector<T> &v)
{
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
template <typename T>
T getStdev(const std::vector<T> &v, const float &mean)
{
    float sum_deviation = 0.0;
    for(size_t i=0; i<v.size(); i++)
    {
        sum_deviation += (v[i]-mean)*(v[i]-mean);
    }
    return std::sqrt(sum_deviation/v.size());
}

template <typename T>
// calculate the standard deviation of the vector
T getStdev(const std::vector<T> &v)
{
    float mean = getMean(v);
    float sum_deviation = 0.0;
    for(size_t i=0; i<v.size(); i++)
    {
        sum_deviation += (v[i]-mean)*(v[i]-mean);
    }
    return std::sqrt(sum_deviation/v.size());
}

#endif // COMMONFUNC_H
