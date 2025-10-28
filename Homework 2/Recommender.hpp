#ifndef RECOMMENDER_HPP
#define RECOMMENDER_HPP

#include <array>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <ctime>

namespace ai {

/**
 * @brief Simple linear regression recommender for menu taste -> predicted satisfaction.
 *
 * We use shape:
 *   y_hat = w0 + w1*x1 + ... + w5*x5
 * where xi corresponds to taste features in order:
 *   [sweet, sour, bitter, salty, savory]
 *
 * The implementation keeps output in 0..10 range (same scale as taste values).
 */
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

    /**
     * @brief Predict satisfaction for given taste vector.
     * @param taste 5-dim taste vector [sweet, sour, bitter, salty, savory]
     * @return predicted satisfaction in 0..10
     */
    double predict(const std::array<double, 5> &taste) const {
        double y = weights[0];
        for (int i = 0; i < 5; ++i)
            y += weights[i + 1] * taste[i];
        if (y < 0) y = 0;
        if (y > 10) y = 10;
        return y;
    }

    /**
     * @brief Update weights using single-step gradient (per assignment).
     * @param taste feature vector
     * @param actual user's actual satisfaction (0..10)
     */
    void update(const std::array<double, 5> &taste, double actual) {
        double predicted = predict(taste);
        double error = actual - predicted;
        // bias update
        weights[0] += learning_rate * error;
        for (int i = 0; i < 5; ++i)
            weights[i + 1] += learning_rate * error * taste[i];
    }

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

    void showWeights() const {
        std::cout << "Current Weights:\n";
        for (int i = 0; i < 6; ++i)
            std::cout << "w" << i << " = " << weights[i] << "\n";
    }
};

} // namespace ai

#endif // RECOMMENDER_HPP
