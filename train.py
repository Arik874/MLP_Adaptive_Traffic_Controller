import numpy as np
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

print("Fixing Feature Ambiguity: Training Ego-Centric AI...")
np.random.seed(42)
num_samples = 50000

# NEW FEATURES: [Active_Demand, Cross_1, Cross_2, Cross_3, Elapsed]
X = np.zeros((num_samples, 5))
y = np.zeros(num_samples)

for i in range(num_samples):
    # 1. Active Lane Demand (0.0 if empty, 5.0-20.0 if cars present)
    active_demand = np.random.choice([0.0, np.random.uniform(5.0, 20.0)])
    
    # 2. Cross Traffic Demands (The 3 red lanes)
    cross_demands = np.random.uniform(low=0.0, high=45.0, size=3)
    
    elapsed = np.random.uniform(low=0.0, high=30.0)
    
    # AI ALWAYS knows X[0] is the green lane
    X[i, 0] = active_demand
    X[i, 1:4] = cross_demands
    X[i, 4] = elapsed
    
    max_cross = np.max(cross_demands)
    sum_cross = np.sum(cross_demands)
    
    # TARGET MATH
    target = 15.0 
    if active_demand == 0.0:
        target = 5.0 # Dead empty lane
    else:
        reward = (active_demand ** 1.2)
        penalty = ((max_cross ** 1.3) * 0.2) + ((sum_cross ** 1.1) * 0.05)
        
        if max_cross > 20.0:
            reward *= 0.1 # Suppress if cross traffic is heavy
            
        target = 15.0 + reward - penalty - (elapsed * 0.5)
    
    if target < 5.0: target = 5.0
    elif target > 30.0: target = 30.0
        
    y[i] = target

print("Scaling data and training the 12x6 TinyMLP...")
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

y_mean = np.mean(y)
y_std = np.std(y)
y_scaled = (y - y_mean) / y_std

mlp = MLPRegressor(hidden_layer_sizes=(12, 6), activation='relu', solver='adam', max_iter=2000, random_state=42)
mlp.fit(X_scaled, y_scaled)

print(f"Training Complete. Model Score (R^2): {mlp.score(X_scaled, y_scaled):.4f}")
print("\n--- COPY AND PASTE THIS ENTIRE OUTPUT INTO: traffic_mlp_weights.c ---\n")

def print_c_array_1d(name, arr):
    values = ", ".join([f"{val:.8f}f" for val in arr])
    print(f"const float {name}[{len(arr)}] = {{{values}}};")

def print_c_array_2d(name, arr):
    print(f"const float {name}[{arr.shape[0]}][{arr.shape[1]}] = {{")
    for i, row in enumerate(arr):
        values = ", ".join([f"{val:.8f}f" for val in row])
        end = "," if i < arr.shape[0] - 1 else ""
        print(f"    {{{values}}}{end}")
    print("};")

print('#include "traffic_mlp_weights.h"\n')
print_c_array_1d("TRAFFIC_FEATURE_MEAN", scaler.mean_)
print_c_array_1d("TRAFFIC_FEATURE_STD", scaler.scale_)
print(f"const float TRAFFIC_TARGET_MEAN = {y_mean:.8f}f;")
print(f"const float TRAFFIC_TARGET_STD = {y_std:.8f}f;\n")

W1, W2, W3 = mlp.coefs_
B1, B2, B3 = mlp.intercepts_

print_c_array_2d("TRAFFIC_W1", W1)
print_c_array_1d("TRAFFIC_B1", B1)
print_c_array_2d("TRAFFIC_W2", W2)
print_c_array_1d("TRAFFIC_B2", B2)
print_c_array_2d("TRAFFIC_W3", W3)
print_c_array_1d("TRAFFIC_B3", B3)
print("\n---------------------------------------------------------")