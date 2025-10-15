import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.svm import SVR
from sklearn.tree import DecisionTreeRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split
import rosbag
from sklearn.preprocessing import MinMaxScaler
import joblib  # Import joblib for saving and loading models

features = np.load("/home/mina/Music/features2.npy")
outputs = np.load("/home/mina/Music/outputs2.npy")

print(f"Features: {np.size(features)}, Outputs: {np.size(outputs)}")
# Split data into train and test sets
x_train, x_test, y_train, y_test = train_test_split(features, outputs, test_size=0.2, random_state=42)

# Models for two outputs
models = {
    "Linear Regression": LinearRegression(),
    "SVM (RBF Kernel)": SVR(kernel="rbf", C=100, gamma=0.1),  # SVM supports only single output
    "Decision Tree": DecisionTreeRegressor(max_depth=10),
    "Neural Network": MLPRegressor(hidden_layer_sizes=(50, 50), max_iter=1000, random_state=42)
}

# Train and evaluate each model
predictions = {}
mse_scores = {}

for name, model in models.items():
    if name == "SVM (RBF Kernel)":  # SVM needs to predict each output separately
        y_pred = np.hstack([
            model.fit(x_train, y_train[:, i]).predict(x_test).reshape(-1, 1)
            for i in range(y_train.shape[1])
        ])
    else:
        model.fit(x_train, y_train)
        y_pred = model.predict(x_test)
    predictions[name] = y_pred
    mse_scores[name] = mean_squared_error(y_test, y_pred)
    if name == "Neural Network":
        print(f"{name} MSE: {mse_scores[name]:.4f}")

# Plot results for each output
fig, axes = plt.subplots(1, 2, figsize=(15, 5))

for i, output_name in enumerate(["Output y1", "Output y2"]):
    #axes[i].plot(y_test[:, i], predictions["Linear Regression"][:, i], label="Linear Regression", alpha=0.6)
    #axes[i].plot(y_test[:, i], predictions["SVM (RBF Kernel)"][:, i], label="SVM", alpha=0.6)
    #axes[i].plot(y_test[:, i], predictions["Decision Tree"][:, i], label="Decision Tree", alpha=0.6)
    axes[i].plot(y_test[:, i], predictions["Neural Network"][:, i], label="Neural Network", alpha=0.6)
    axes[i].set_title(f"True vs Predicted ({output_name})")
    axes[i].set_xlabel("True Value")
    axes[i].set_ylabel("Predicted Value")
    axes[i].legend()
    axes[i].grid()
    
for name, model in models.items():
    if name == "Neural Network":
        model_file = f"{name.replace(' ', '_')}_model.pkl"
        joblib.dump(model, model_file)
        print(f"Model {name} saved to {model_file}")

plt.tight_layout()
plt.show()
