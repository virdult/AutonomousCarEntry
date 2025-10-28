#ifndef RECOMMENDER_HPP
#define RECOMMENDER_HPP

#include <array>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <ctime>

namespace ai {

// Simple linear regression recommender for menu taste -> predicted satisfaction.
// Model: y_hat = w0 + w1*x1 + ... + w5*x5
// Taste features: [sweet, sour, bitter, salty, savory]
// Output is kept within 0..10 range.
class Recommender {
private:
    std::array<double, 6> weights; // w0..w5
    double learning_rate;

public:
    Recommender(double lr = 0.01) : learning_rate(lr) {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        for (double &w : weights) {
            w = ((double) std::rand() / RAND_MAX) * 0.1;
        }
    }

    // Predict satisfaction for a given taste vector.
    double predict(const std::array<double, 5> &taste) const {
        double y = weights[0];
        for (int i = 0; i < 5; ++i)
            y += weights[i + 1] * taste[i];
        if (y < 0) y = 0;
        if (y > 10) y = 10;
        return y;
    }

    // Update weights using a single-step gradient.
    void update(const std::array<double, 5> &taste, double actual) {
        double predicted = predict(taste);
        double error = actual - predicted;
        weights[0] += learning_rate * error; // bias
        for (int i = 0; i < 5; ++i)
            weights[i + 1] += learning_rate * error * taste[i];
    }

    // Save current weights to a JSON file.
    void saveWeights(const std::string &filename = "weights.json") const {
        nlohmann::json j;
        j["weights"] = weights;
        std::ofstream file(filename);
        if (file.is_open()) {
            file << j.dump(4);
            file.close();
            std::cout << "Weights saved to " << filename << "\n";
        } else {
            std::cerr << "Failed to save weights!\n";
        }
    }

    // Load weights from a JSON file (if available).
    void loadWeights(const std::string &filename = "weights.json") {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "No saved weights found. Starting fresh.\n";
            return;
        }
        nlohmann::json j;
        file >> j;
        file.close();
        if (j.contains("weights") && j["weights"].is_array()) {
            weights = j["weights"].get<std::array<double, 6>>();
            std::cout << "Weights loaded successfully.\n";
        } else {
            std::cerr << "Weights file malformed - starting fresh.\n";
        }
    }

    // Print current weight values.
    void showWeights() const {
        std::cout << "Current Weights:\n";
        for (int i = 0; i < 6; ++i)
            std::cout << "w" << i << " = " << weights[i] << "\n";
    }
};

} // namespace ai

#endif // RECOMMENDER_HPP
