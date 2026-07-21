import os
import cv2
import numpy as np
import joblib
from skimage.feature import hog
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

print("Initializing SVM Training Pipeline...")

# 1. Configuration
DATASET_DIR = "custom_dataset"
CATEGORIES = ["background", "target_object"]
IMG_SIZE = (64, 64)

data = []
labels = []

# 2. Extract Features using HOG
print("Extracting HOG features from images...")
for category in CATEGORIES:
    path = os.path.join(DATASET_DIR, category)
    if not os.path.exists(path):
        print(f"ERROR: Could not find folder {path}")
        continue
        
    class_num = CATEGORIES.index(category) 
    
    for img_name in os.listdir(path):
        try:
            img_path = os.path.join(path, img_name)
            img = cv2.imread(img_path)
            if img is None: continue # Skip non-images
            
            # Convert to grayscale for shape detection
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            resized = cv2.resize(gray, IMG_SIZE)
            
            # Extract HOG features (captures gradients and edges)
            features = hog(resized, orientations=9, pixels_per_cell=(8, 8), 
                           cells_per_block=(2, 2), visualize=False)
            
            data.append(features)
            labels.append(class_num)
        except Exception as e:
            print(f"Skipping {img_name}: {e}")

# Convert lists to NumPy arrays for scikit-learn
X = np.array(data)
y = np.array(labels)

if len(X) == 0:
    print("ERROR: No images found. Did you save them in custom_dataset?")
else:
    # 3. Train/Test Split (80% training, 20% testing)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # 4. Train the Support Vector Machine
    print(f"Training SVM on {len(X_train)} samples...")
    svm_model = SVC(kernel='linear', probability=True)
    svm_model.fit(X_train, y_train)

    # 5. Evaluate the Model
    predictions = svm_model.predict(X_test)
    accuracy = accuracy_score(y_test, predictions)
    print(f"\n>>> Model Validation Accuracy: {accuracy * 100:.2f}%")

    # 6. Export the Weights
    model_filename = "drone_target_svm.pkl"
    joblib.dump(svm_model, model_filename)
    print(f"Custom AI weights saved to {model_filename}")