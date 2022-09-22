#ifndef REGULATED_PURE_PURSUIT_HELPER_
#define REGULATED_PURE_PURSUIT_HELPER_

#include <utility>
#include <cmath>

class RegulatedPurePursuitHelper
{
public:
    RegulatedPurePursuitHelper(double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y);
    ~RegulatedPurePursuitHelper();
    double inverseCosineVector();
    double inverseCosineVectorinDegrees();
    double cosineVector();
    void obtainA();
    void obtainB();
    double dotProduct();
    double magnitudeOfVector();
    std::pair<double, double> vector_a_{0.0, 0.0};
    std::pair<double, double> vector_b_{0.0, 0.0};

    // Private functions are not directly accessible inverseCosineVector and only accessible via public methods
private:
    double p1_x_{0.0};
    double p1_y_{0.0};
    double p2_x_{0.0};
    double p2_y_{0.0};
    double p3_x_{0.0};
    double p3_y_{0.0};

    double angle_{0.0};
};

#endif // REGULATED_PURE_PURSUIT_HELPER_
