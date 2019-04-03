/**
 * @file quad_matcher.h
 * @author Huimin Cheng (NUS)
 * @brief Perform circular match on 4 sets of unmatched features. Header-only implementation, required by templated programming
 * @version 0.1
 * @date 2018-10-02
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#ifndef QUAD_MATCHER_H
#define QUAD_MATCHER_H

#define UNUSED(x) (void)(x)

// OpenCV
#include <opencv2/core.hpp>

#include <vector>
#include <list>
#include <array>
#include <algorithm>

// implementation
#include <iostream>
#include <cassert>

#include "viso2_eigen_params.h"

// Class TDescriptor should be of integer type, able to perform (query^(*targets)[i]).count()
// Class TFeature should implement function: int distance(const TDescriptor &a, const TDescriptor &b)
// TODO: TFeature is NOT used anymore, time to clear

template<class TDescriptor, class TFeature>
class QuadMatcher{

public:
    typedef std::array<int,4> DMatch; // l1, l2, r2, r1

    QuadMatcher();

    void setParam(QuadMatcherParam::Parameters &param);

    void bucketKeyPoints(std::vector<cv::KeyPoint> &key, double scale=1.0);

    void pushBackData(const std::vector<cv::KeyPoint> &keyl1, const std::vector<cv::KeyPoint> &keyl2, 
                            const std::vector<cv::KeyPoint> &keyr1, const std::vector<cv::KeyPoint> &keyr2,
                            const std::vector<TDescriptor> &desl1, const std::vector<TDescriptor> &desl2,
                            const std::vector<TDescriptor> &desr1, const std::vector<TDescriptor> &desr2 );
    
    // Circular matching of 4 images
    // 1. previous left --> current left
    // 2. current left --> current right
    // 3. current right --> previous right
    // 4. previous right --> previous left

    bool matchFeaturesQuad();

    void getMatchesQuad( std::vector< DMatch > &matches_quad );
private:

    bool initialised;

    // Indicate whether to use epipolar constraints (if enabled) to bucket points before matching (so less points to match)
    enum HorizontalConstraint {NONE, LEFT_ONLY, RIGHT_ONLY};

    // Signal indicating that the quad image sequences are loaded, and
    // ready to be processed for matching and RT estimation.
    bool data_ready;

    // Containing all the tunable configurations
    QuadMatcherParam::Parameters param;

    // Maintain a vector list for each of the four image sequences, for indexing points in each bucket block
    std::vector< std::vector<int> > bucketl1, bucketr1, bucketl2, bucketr2;

    // Bucket index is ROW MAJOR
    inline int getBucketIndex(float x, float y);
    inline std::vector<int> getEpipolarBucketPoints(const cv::KeyPoint &key, 
                                                    const std::vector< std::vector<int> > &bucket,
                                                    const HorizontalConstraint constraint);
    void createBucketIndices(const std::vector<cv::KeyPoint> *keys , 
                                std::vector< std::vector<int> > &bucket,
                                double scale = 1.0);

    
    const double maximum_distance_ratio = 0.3;
    inline int findMatch(   const TDescriptor &query, const std::vector<TDescriptor> *targets );
    inline int findMatch(   const std::vector<int> &inside_bucket, const TDescriptor &query,
                            const std::vector<TDescriptor> *targets );

    
    void updateMatchList( const int matches_source, const int matches_target,
                    const std::vector<cv::KeyPoint> *key_sources, 
                    const std::vector< std::vector<int> > &bucket,
                    const std::vector<cv::KeyPoint> *key_targets,
                    const std::vector<TDescriptor> *des_sources, 
                    const std::vector<TDescriptor> *des_targets, 
                    const HorizontalConstraint constraint );
    
    
    // Only store pointers to the actual data of keypoints and their descriptors to avoid copy overhead
    // NOTE: This class does not store data, so make sure the data is in scope throughout each processing iteration 
    const std::vector<cv::KeyPoint> *keyl1, *keyl2, *keyr1, *keyr2;
    const std::vector<TDescriptor> *desl1, *desl2, *desr1, *desr2;

    // Matching results
    std::list< std::array<int,5> > matches;
};

/////////////////////////////////////////////////////////////////////////////////////////////////
//// IMPLEMENTATION STARTS HERE
/////////////////////////////////////////////////////////////////////////////////////////////////

int distance_counter = 0;

template<class TDescriptor, class TFeature>
QuadMatcher<TDescriptor, TFeature>::QuadMatcher() : initialised(false), data_ready(false) {
    int num_bucket = param.n_bucket_width * param.n_bucket_height;
    // Reserve # of buckets space in the bucket vector
    bucketl1.resize(num_bucket);
    bucketr1.resize(num_bucket);
    bucketl2.resize(num_bucket);
    bucketr2.resize(num_bucket);
}

///////////////////////////////////////////////////////////////////////
//// Implementation of Public Member Functions
///////////////////////////////////////////////////////////////////////

template<class TDescriptor, class TFeature>
void QuadMatcher<TDescriptor, TFeature>::setParam(QuadMatcherParam::Parameters &param){

    // calculating the suitable bucket size for the given 2D bucket dimensions
    param.bucket_height = (param.image_height + param.n_bucket_height - 1 ) / param.n_bucket_height;
    param.bucket_width = (param.image_width + param.n_bucket_width - 1 ) / param.n_bucket_width;

    std::cout << std::endl << "QuadMatcher parameters loaded!" << std::endl;
    std::cout << "- image size " << param.image_width << "x" << param.image_height << std::endl
        << "- bucketing " << param.n_bucket_width << "x" << param.n_bucket_height 
        << " ["<< param.bucket_width << "," << param.bucket_height << "]" << std::endl;

    std::cout << "==============================================================" << std::endl;
    
    this->param = param;
    initialised = true;
}

bool cmpResponse(cv::KeyPoint i, cv::KeyPoint j){
    return i.response > j.response;
}

template<class TDescriptor, class TFeature>
void QuadMatcher<TDescriptor, TFeature>::bucketKeyPoints(std::vector<cv::KeyPoint> &key, double scale){

    const size_t Nmax = param.max_features_per_bucket;

    if (Nmax == 0) // feature disabled
        return;

    std::vector<cv::KeyPoint> result_key;

    std::vector< std::vector<int> > bucket(param.n_bucket_width * param.n_bucket_height);

    createBucketIndices(&key, bucket, scale);

    size_t total_key = 0;
    for (size_t i =0; i < bucket.size(); i++){

        std::vector<cv::KeyPoint> bucket_key;

        // for each index in the bucket, add the key to a temporary vector for sorting
        for ( int idx : bucket[i]){
            bucket_key.push_back(key[idx]);
        }

        total_key += bucket_key.size();

        // sort process
        std::sort(bucket_key.begin(),bucket_key.end(), cmpResponse);

        int upper_limit = std::min(Nmax,bucket_key.size());

        result_key.insert(result_key.end(),bucket_key.begin(),bucket_key.begin() + upper_limit);
    }

    assert(total_key == key.size());

    std::cout << "Bucketing KeyPoints from " << total_key << " to " << result_key.size() << std::endl;

    key = result_key;
}

// l1 - previous left (match), l2 current left (query)
template<class TDescriptor, class TFeature>
void QuadMatcher<TDescriptor, TFeature>::pushBackData(
                            const std::vector<cv::KeyPoint> &keyl1, 
                            const std::vector<cv::KeyPoint> &keyl2, 
                            const std::vector<cv::KeyPoint> &keyr1, 
                            const std::vector<cv::KeyPoint> &keyr2, 
                            const std::vector<TDescriptor> &desl1, 
                            const std::vector<TDescriptor> &desl2,
                            const std::vector<TDescriptor> &desr1, 
                            const std::vector<TDescriptor> &desr2 ) {

    if (!initialised)
    {
        std::cerr << "QuadMatcher NOT initialised." << std::endl;
        return;
    }
        
    // Assignment to pointers
    this->keyl1 = &keyl1;
    this->keyr1 = &keyr1;
    this->keyl2 = &keyl2;
    this->keyr2 = &keyr2;

    this->desl1 = &desl1;
    this->desr1 = &desr1;
    this->desl2 = &desl2;
    this->desr2 = &desr2;

    assert( !keyl1.empty());
    assert( !keyl2.empty());
    assert( !keyr1.empty());
    assert( !keyr2.empty());

    assert( keyl1.size() == desl1.size() );
    assert( keyr1.size() == desr1.size() );
    assert( keyl2.size() == desl2.size() );
    assert( keyr2.size() == desr2.size() );

    // std::cout << "Creating Bucket Indices" << std::endl;

    createBucketIndices(this->keyl1,bucketl1);
    createBucketIndices(this->keyr1,bucketr1);
    createBucketIndices(this->keyl2,bucketl2);
    createBucketIndices(this->keyr2,bucketr2);

    data_ready = true;
}



template<class TDescriptor, class TFeature>
bool QuadMatcher<TDescriptor, TFeature>::matchFeaturesQuad() {
    if (!data_ready)
    {
        std::cerr << "QuadMatcher ERROR: Data not data_ready." << std::endl;
        return false;
    }

    // cv::BFMatcher matcher;

    // Initialise the matches list with all points from 
    std::cout << "main: query # keys: " << keyl2->size() << ", match # keys: " << keyl1->size() << std::endl;
    std::cout << "right: query # keys: " << keyr2->size() << ", match # keys: " << keyl1->size() << std::endl;

    matches.clear();

    // int pre = distance_counter;
    // TODO: for the NONE case, the homographic transformation could be estimated after a few initial match
    updateMatchList (0, 1, keyl1, bucketl2, keyl2, desl1, desl2, NONE); // previous left --> current left
    int initial_match_size = matches.size();
    updateMatchList (1, 2, keyl2, bucketr2, keyr2, desl2, desr2, LEFT_ONLY); // current left --> current right
    updateMatchList (2, 3, keyr2, bucketr1, keyr1, desr2, desr1, NONE); // current right --> previous right
    updateMatchList (3, 4, keyr1, bucketl1, keyl1, desr1, desl1, RIGHT_ONLY); // previous right --> previous left

        

    // std::cout << "distance() routine is called " << distance_counter - pre <<  " times" << std::endl;
    

    // int list_size = matches.size();

    // Clean up wrong looped matches
    auto it = matches.begin();
    while (it != matches.end())
    {
        if ( (*it)[0] != (*it)[4]) // matching loop failed
        {
            it = matches.erase(it);
        }else
            it++;
    }
    std::cout << "Matching loop closes: " << matches.size() << " out of " << initial_match_size << "(l1->l2)"<< std::endl;
    
    data_ready = false;

    // We need at least 6 points to make meaningful calculations
    return (matches.size() > 5);
}



template<class TDescriptor, class TFeature>
void QuadMatcher<TDescriptor, TFeature>::getMatchesQuad( std::vector< DMatch > &matches_quad ) {
    
    assert (matches_quad.empty());

    matches_quad.reserve(matches.size());

    for ( auto match : matches )
    {
        matches_quad.push_back( DMatch{match[0], match[1], match[2], match[3]} );
    }

}



///////////////////////////////////////////////////////////////////////
//// Implementation of Private Member Functions
///////////////////////////////////////////////////////////////////////



// OpenCV use top-left as origin, x as column/width, y as row/height
template<class TDescriptor, class TFeature>
int QuadMatcher<TDescriptor, TFeature>::getBucketIndex(float x, float y) {
    
    const int idx_y = y / param.bucket_height;
    const int idx_x = x / param.bucket_width;

    if ( !(idx_y >= 0 && idx_y < param.n_bucket_height && idx_x >= 0 && idx_x < param.n_bucket_width) )
    {
        std::cout << "x = " << x <<", idx_x = " << idx_x << ", y = " << y <<", idx_y = " << idx_y << std::endl;
        exit(-1);
    }

    int idx = idx_y*param.n_bucket_width + idx_x;
    assert (idx >= 0 && idx < param.n_bucket_height*param.n_bucket_width);

    return idx;
}

template<class TDescriptor, class TFeature>
std::vector<int> QuadMatcher<TDescriptor, TFeature>::getEpipolarBucketPoints(
        const cv::KeyPoint &key, const std::vector< std::vector<int> > &bucket, const HorizontalConstraint constraint)
{

    // Determine offset sign based on left-right or right-left matching
    const int offset = (constraint == LEFT_ONLY) ? param.epipolar_offset : - param.epipolar_offset; // RIGHT_ONLY means left->right matching

    // Assume ROW MAJOR bucketing

    const float lower_bound_y = std::max( 0.f  , key.pt.y + offset - param.epipolar_tolerance) ; // top-left origin x == width direction, https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column
    const float upper_bound_y = std::min( param.image_height - 1.f, key.pt.y + offset + param.epipolar_tolerance );

    const float lower_bound_x = ( constraint == RIGHT_ONLY ? key.pt.x : 0);
    const float upper_bound_x = ( constraint == LEFT_ONLY ? key.pt.x : param.image_width-1 );

    const int lower_bound_bucket = getBucketIndex( lower_bound_x, lower_bound_y);
    const int upper_bound_bucket = getBucketIndex( upper_bound_x , upper_bound_y);

    std::vector<int> results;

    for (int i = lower_bound_bucket ; i <= upper_bound_bucket; i++)
    {
        for ( auto e : bucket[i] )
        {
            results.push_back(e);
        }
    }

    return results;
}

template<class TDescriptor, class TFeature>
void QuadMatcher<TDescriptor, TFeature>::createBucketIndices(const std::vector<cv::KeyPoint> *keys , 
                                                    std::vector< std::vector<int> > &bucket,
                                                    double scale) {
    
    
    assert ( (int)bucket.size() == param.n_bucket_width * param.n_bucket_height);

    // Clear the previous bucket cache
    for ( auto &element : bucket )
        element.clear();

    // Iterate all keypoints and add them to the correct bucket list
    for (size_t i = 0 ; i < keys->size() ; i++)
    {
        const cv::KeyPoint &key = (*keys)[i];
        int idx = getBucketIndex(key.pt.x*scale, key.pt.y*scale);
        bucket[idx].push_back(i);
    }

    // Debug Output
    // for (int i = 0 ; i < bucket.size() ; i++)
    // {
    //     std::cout << "Bucket [" << i << "]:" << std::endl;
    //     for (int j = 0; j < bucket[i].size() ; j++)
    //     {
    //         cout << bucket[i][j] << ":(" << (*keys)[ bucket[i][j] ].pt.x << "," << (*keys)[ bucket[i][j] ].pt.y << ") ";
    //     }
    //     std::cout << std::endl;
    // }
    
}

template<class TDescriptor, class TFeature>
int QuadMatcher<TDescriptor, TFeature>::findMatch(const TDescriptor &query,
                            const std::vector<TDescriptor> *targets )
{
    int best_dist_1 = 1e9;
    int best_dist_2 = 1e9;
    int best_i = -1;

    //// Not using bucketing
    for (size_t i = 0; i < targets->size(); i++)
    {
        int dist = (query^(*targets)[i]).count(); //TFeature::distance(query,(*targets)[i]);
        distance_counter++;

        if (dist < best_dist_1 ) {
            best_i = i;
            best_dist_2 = best_dist_1;
            best_dist_1 = dist;
        }else if (dist < best_dist_2) {
            best_dist_2 = dist;
        }
    }

    // If the best match is much better than the second best, then it is most probably a good match (SNR good)
    if ( best_dist_1 / (double)best_dist_2 < param.max_neighbor_ratio)
    {
        if (maximum_distance_ratio * query.size() > best_dist_1)
            return best_i;
        else
            return -1; // even the best candidate has a distance too large
    }
    return -1;
}

template<class TDescriptor, class TFeature>
int QuadMatcher<TDescriptor, TFeature>::findMatch( const std::vector<int> &inside_bucket, const TDescriptor &query,
                            const std::vector<TDescriptor> *targets )
{
    
    int best_dist_1 = 1e9;
    int best_dist_2 = 1e9;
    int best_i = -1;

    // there is no suitable bucketed target points to be matched, early return
    if (inside_bucket.empty())
        return -1;

    // std::cout << "Bucketing " << inside_bucket.size() << " target features out of " << targets->size() << std::endl;

    for ( size_t idx = 0 ; idx < inside_bucket.size() ; idx++ )
    {
        int i = inside_bucket[idx];

        int dist = (query^(*targets)[i]).count(); //TFeature::distance(query,(*targets)[i]);
        distance_counter++;

        if (dist < best_dist_1 ) {
            best_i = i;
            best_dist_2 = best_dist_1;
            best_dist_1 = dist;
        }else if (dist < best_dist_2) {
            best_dist_2 = dist;
        }
    }

    // If the best match is much better than the second best, then it is most probably a good match (SNR good)
    if ( best_dist_1 / (double)best_dist_2 < param.max_neighbor_ratio)
    {
        if (maximum_distance_ratio * query.size() > best_dist_1)
            return best_i;
        else
            return -1; // even the best candidate has a distance too large
    }
    return -1;
}

template<class TDescriptor, class TFeature>
void QuadMatcher<TDescriptor, TFeature>::updateMatchList( 
                const int matches_source, 
                const int matches_target,
                const std::vector<cv::KeyPoint> *key_sources, 
                const std::vector< std::vector<int> > &bucket,
                const std::vector<cv::KeyPoint> *key_targets,
                const std::vector<TDescriptor> *des_sources, 
                const std::vector<TDescriptor> *des_targets,
                const HorizontalConstraint constraint  ) {

    UNUSED(key_targets);

    // if matches_source is zero, means the match list needs initialisation
    if (matches_source == 0) {

        assert (matches.empty());

        for ( size_t i = 0; i < (*key_sources).size() ; i++ )
        {
            const TDescriptor &des = (*des_sources)[i];
            const cv::KeyPoint &key = (*key_sources)[i];
            int j;
            if (param.use_bucketing && constraint != NONE) 
                j = findMatch( getEpipolarBucketPoints(key, bucket, constraint), des, des_targets );
            else
                j = findMatch( des, des_targets );

            if (j >= 0)
            {
                matches.push_back({(int)i,j,0,0,0});
            }
        }

        // std::cout << "Found " << matches.size() << " matches between 0 and 1" << std::endl;
    }
    // else, use the [matches_source] column as the query points
    else {

        // int list_size = matches.size();

        auto it = matches.begin();
        while(it != matches.end())
        {
            int i = (*it)[matches_source]; // iterated index of current keypoints / descriptors

            const TDescriptor &des = (*des_sources)[i];
            const cv::KeyPoint &key = (*key_sources)[i];
            int j;
            if (param.use_bucketing && constraint != NONE) 
                j = findMatch( getEpipolarBucketPoints(key, bucket, constraint), des, des_targets );
            else
                j = findMatch( des, des_targets );

            // Found matches for ith keypoint in the source image to the jth keypoint in the target image
            if (j >= 0)
            {
                (*it)[matches_target] = j;
                it++;
            }
            // failure to find, delete this node, and proceed to the next node
            else
            {
                it = matches.erase(it);
            }
        }

        // std::cout << "Matches " << matches.size() << " out of previous " << list_size << " from " 
        //     <<  matches_source << " to " << matches_target << std::endl;
    }
}

#endif // QUAD_MATCHER_H