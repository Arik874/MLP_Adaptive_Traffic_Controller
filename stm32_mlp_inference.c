#include "stm32_mlp_inference.h"
#include "traffic_mlp_weights.h"

// UPGRADED ARCHITECTURE DEFINITIONS
#define INPUT_NODES 5
#define HIDDEN1_NODES 12
#define HIDDEN2_NODES 6
#define OUTPUT_NODES 1

#define MIN_GREEN_SECONDS 5.0f
#define MAX_GREEN_SECONDS 30.0f

// ReLU Activation Function
static inline float relu(float x) {
    return (x > 0.0f) ? x : 0.0f;
}

float traffic_mlp_predict_seconds(const float input[INPUT_NODES]) {
    float scaled_input[INPUT_NODES];
    float h1[HIDDEN1_NODES];
    float h2[HIDDEN2_NODES];
    float output = 0.0f;

    // 1. Scale Inputs
    for (int i = 0; i < INPUT_NODES; i++) {
        scaled_input[i] = (input[i] - TRAFFIC_FEATURE_MEAN[i]) / TRAFFIC_FEATURE_STD[i];
    }

    // 2. Hidden Layer 1 (12 Neurons)
    for (int i = 0; i < HIDDEN1_NODES; i++) {
        h1[i] = TRAFFIC_B1[i];
        for (int j = 0; j < INPUT_NODES; j++) {
            h1[i] += scaled_input[j] * TRAFFIC_W1[j][i];
        }
        h1[i] = relu(h1[i]);
    }

    // 3. Hidden Layer 2 (6 Neurons)
    for (int i = 0; i < HIDDEN2_NODES; i++) {
        h2[i] = TRAFFIC_B2[i];
        for (int j = 0; j < HIDDEN1_NODES; j++) {
            h2[i] += h1[j] * TRAFFIC_W2[j][i];
        }
        h2[i] = relu(h2[i]);
    }

    // 4. Output Layer
    output = TRAFFIC_B3[0];
    for (int j = 0; j < HIDDEN2_NODES; j++) {
        output += h2[j] * TRAFFIC_W3[j][0];
    }

    // 5. Unscale Output
    float final_seconds = (output * TRAFFIC_TARGET_STD) + TRAFFIC_TARGET_MEAN;

    // 6. Safety Clamp
    if (final_seconds < MIN_GREEN_SECONDS) final_seconds = MIN_GREEN_SECONDS;
    if (final_seconds > MAX_GREEN_SECONDS) final_seconds = MAX_GREEN_SECONDS;

    return final_seconds;
}