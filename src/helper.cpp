#include "regulated_pure_pursuit_controller/helper.h"

RegulatedPurePursuitHelper::RegulatedPurePursuitHelper(double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y) : p1_x_(p1_x),
                                                                                                                                       p1_y_(p1_y),
                                                                                                                                       p2_x_(p2_x),
                                                                                                                                       p2_y_(p2_y),
                                                                                                                                       p3_x_(p3_x),
                                                                                                                                       p3_y_(p3_y)
{
    // This will calcuate the values of vector A and B
    obtainA();
    obtainB();
    cosineVector();
}
RegulatedPurePursuitHelper::~RegulatedPurePursuitHelper() {}

void RegulatedPurePursuitHelper::obtainA()
{
    // Create a vector consisting of 2 double values
    vector_a_ = std::make_pair(p1_x_ - p2_x_, p1_y_ - p2_y_);
    return;
}

void RegulatedPurePursuitHelper::obtainB()
{
    // Create a vector consisting of 2 double values
    vector_b_ = std::make_pair(p3_x_ - p2_x_, p3_y_ - p2_y_);
    return;
}

double RegulatedPurePursuitHelper::dotProduct()
{
    double answer = vector_a_.first * vector_b_.first + vector_a_.second * vector_b_.second;
    return answer;
}

double RegulatedPurePursuitHelper::magnitudeOfVector()
{
    double magnitude_of_a = std::sqrt(std::pow(vector_a_.first, 2) + std::pow(vector_a_.second, 2));
    double magnitude_of_b = std::sqrt(std::pow(vector_b_.first, 2) + std::pow(vector_b_.second, 2));
    double answer = magnitude_of_a * magnitude_of_b;
    return answer;
}

double RegulatedPurePursuitHelper::cosineVector()
{
    double dot_product = dotProduct();
    double magnitude_of_vector = magnitudeOfVector();
    angle_ = dot_product / magnitude_of_vector;
    return angle_;
}

double RegulatedPurePursuitHelper::inverseCosineVector()
{
    return std::acos(angle_);
}

double RegulatedPurePursuitHelper::inverseCosineVectorinDegrees()
{
    return std::acos(angle_) * 180.0 / M_PI;
}
