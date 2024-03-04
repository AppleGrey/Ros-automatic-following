#pragma once
#include <iostream>
#include <vector>
#include <cmath>

static double calculateMean(const std::vector<double>& data) {
    double sum = 0.0;
    for (const auto& value : data) {
        sum += value;
    }
    return sum / data.size();
}

static double calculateStandardDeviation(const std::vector<double>& data, double mean) {
    double sumOfSquares = 0.0;
    for (const auto& value : data) {
        double diff = value - mean;
        sumOfSquares += diff * diff;
    }
    return sqrt(sumOfSquares / data.size());
}

static int grubbsTest(const std::vector<double>& data, std::vector<int>& outlierIndices) {
    if (data.empty())
        return -1;

    int n = data.size();
    double mean = calculateMean(data);
    double sd = calculateStandardDeviation(data, mean);

    if (sd == 0.0)
        return -1;

    double criticalValue = 1.15;  // Assuming 95% confidence level, adjust if needed

    outlierIndices.clear();

    for (int i = 0; i < n; ++i) {
        double g = std::abs(data[i] - mean) / sd;
        if (g > criticalValue) {
            outlierIndices.push_back(i);
        }
    }

    return outlierIndices.size();
}

/**
 * remove error values
*/
static void grubbsFilter(std::vector<double>& data) {
    std::vector<int> outlierIndices;
    int numOutliers = grubbsTest(data, outlierIndices);

    if (numOutliers > 0) {
        for (int i = numOutliers - 1; i >= 0; --i) {
            int outlierIndex = outlierIndices[i];
            data.erase(data.begin() + outlierIndex);
        }
    }

}