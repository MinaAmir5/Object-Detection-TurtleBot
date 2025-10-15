import cv2
import numpy as np
import math
from ultralytics import YOLO
import rospy
from std_msgs.msg import Float32MultiArray
import torch

# Load the YOLO model (replace with your model path if custom-trained)
model = YOLO("best.pt")  # YOLOv8 pre-trained model
if torch.cuda.is_available():
    model.to('cuda')
else:
    print("CUDA is not available. Model is using CPU.")

print(model.device)  # Should show "cuda" if the model is running on GPU# IP camera stream URL# Define camera matrix and distortion coefficients (from calibration)
camera_matrix = np.array([[425.88745301, 0, 245.73935418], [0, 424.90191586, 365.30223806], [0, 0, 1]], dtype=float)
dist_coeffs = np.array([2.64425182e-01, -1.29664331e+00, 5.01915095e-05, 7.78248239e-05, 2.66288798e+00], dtype=float)  # Update with actual distortion coefficients

# IP Camera Stream
ip_camera_url = "http://192.168.1.15:8080/video"  # Replace with actual URL
# Open video stream
cap = cv2.VideoCapture(ip_camera_url)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()
    
rospy.init_node('camera', anonymous=True)
trash_publisher = rospy.Publisher('trash', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)
trash = Float32MultiArray() 

scaling_factor = 0.12  # Replace with actual scaling based on calibration
# Define the target class index for "bottle" (COCO dataset)
bottle_class_index = 39
person_class_index = 0

def draw_dashed_line(img, start_point, end_point, color, thickness=1, dash_length=10):
    """Draws a dashed line from start_point to end_point."""
    x1, y1 = start_point
    x2, y2 = end_point
    distance = int(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    for i in range(0, distance, 2 * dash_length):
        start_x = int(x1 + (x2 - x1) * i / distance)
        start_y = int(y1 + (y2 - y1) * i / distance)
        end_x = int(x1 + (x2 - x1) * (i + dash_length) / distance)
        end_y = int(y1 + (y2 - y1) * (i + dash_length) / distance)
        cv2.line(img, (start_x, start_y), (end_x, end_y), color, thickness)

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()
    if not ret:
        break

    # Resize frame to speed up processing
    frame_resized = cv2.resize(frame, (640, 480))
    height, width, _ = frame_resized.shape

    # Calculate camera center
    camera_center = (width // 2, height )
    cv2.circle(frame_resized, camera_center, radius=10, color=(0, 0, 255), thickness=-2)
    # Run YOLO inference (using stream=True for faster processing)
    results = model(frame_resized, stream=True)

    closest_distance = 1000
    # Process YOLO results
    for result in results:
        #if(closest_distance < 20):
        #    closest_distance = 1000
        #    closest_angle = 0
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # Bounding box [x1, y1, x2, y2]
            cls = int(box.cls.cpu().numpy())  # Class index

            # Get the center of the bounding box
            x_center = int((bbox[0] + bbox[2]) / 2)
            y_center = int((bbox[1] + bbox[3]) / 2)
            object_center = (x_center, y_center)

            # Draw a dashed line from the camera center to the object center
            draw_dashed_line(frame_resized, camera_center, object_center, color=(0, 255, 255), thickness=2)

            # Calculate distance and angle
            dx = x_center - camera_center[0]
            dy = y_center - camera_center[1]
            box_width = bbox[2] - bbox[0]
            distance = ((camera_matrix[0, 0] * scaling_factor) / box_width)*100
            #distance = math.sqrt(dx ** 2 + dy ** 2)  # Pixel distance
            angle = 90-abs(math.degrees(math.atan2(dy, dx)))  # Angle in degrees
            
            if(distance<closest_distance):
                closest_distance = distance
                closest_angle = angle

            # Display distance and angle
            label = f"Obj: {arr[cls]},Dist: {distance:.2f}cm, Angle: {angle:.2f}deg"
            print(label)
            cv2.putText(frame_resized, label, (x_center, y_center - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # Display the frame
    cv2.imshow("Yolo Detection", frame_resized)
    if(closest_distance != 1000):
        trash.data = [closest_distance, closest_angle, cls]
        trash_publisher.publish(trash)
        rospy.loginfo(f"closest distance: {closest_distance}, closest angle: {closest_angle}, class index: {cls}")
    
    
    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources after completion
cap.release()
cv2.destroyAllWindows()
