#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
      TODO:
        * Calculate the RMSE here.
        * note:
        *   code taken from answer at 5.23
      */

    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    if (estimations.size() == 0) {
        cout << "Error: estimations size 0\n";
        return rmse;
    }
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()) {
        cout << "Error: estimations size not equal to ground truth size\n";
    }

    //accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        for (int j = 0; j < 4; ++j) {
            rmse(j) += pow(estimations[i](j) - ground_truth[i](j), 2);
        }
    }

    //calculate the mean
    for (int j = 0; j < 4; ++j) {
        rmse(j) /= estimations.size();
    }

    //calculate the squared root
    for (int j = 0; j < 4; ++j) {
        rmse(j) = sqrt(rmse(j));
    }

    //return the result
    return rmse;

}