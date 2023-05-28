# T2Aux_Robotica
Código

```Python
import cv2
import numpy as np

# Open the video file
video_path = "Tareas/t2_apples.mp4"
cap = cv2.VideoCapture(video_path)

# Check if the video file was successfully opened
if not cap.isOpened():
    print("Error opening video file")

# Read the first frame
ret, frame = cap.read()

def frame_processing(frame):
        #------------  ADD YOUR CODE HERE   -----------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convertir la imagen de RGB a HSV

        lower_red = np.array([0, 100, 80])    # Limite inferior color rojo HSV
        upper_red = np.array([10, 255, 255])  # Limite superior color rojo HSV
        
        lower_green = np.array([24, 120, 120]) # Limite inferior color verde HSV
        upper_green = np.array([75, 255, 255]) # Limite superior color verde HSV

        mask_red = cv2.inRange(hsv, lower_red, upper_red)      # Generar mascara color rojo
        mask_green = cv2.inRange(hsv,lower_green, upper_green) # Generar mascara color verde

        kernel = np.ones((5,5),np.uint8)      # Kernel a utilizar
        dilated_imageRed = cv2.dilate(mask_red, kernel, iterations = 2)     # Dilatar imagen con respecto al color rojo
        dilated_imageGreen = cv2.dilate(mask_green, kernel, iterations = 2) # Dilatar imagen con respecto al color verde

        contours_Red, _ = cv2.findContours(dilated_imageRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)     
        contours_Green, _ = cv2.findContours(dilated_imageGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        color_rojo=(0,0,255)    # Color rojo RGB
        color_verde=(0,255,0)   # Color verde RGB

        
        for cnt in contours_Red:         # Crear bounding boxes rojas para las manzanas rojas
            area = cv2.contourArea(cnt)
            if area > 1200:  # Ignore small contours
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color_rojo, 2)
                
        for cnt in contours_Green:       # Crear bounding boxes verdes para las manzanas verdes
            area = cv2.contourArea(cnt)
            if area > 1200:  # Ignore small contours
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color_verde, 2)
                
        new_frame = frame
        #------------ END -----------
        return new_frame


new_video = []

# Loop through all frames
while ret:
    # Process the frame here    
    new_frame = frame_processing(frame)

    # Add new frame to list of new video
    new_video.append(new_frame)

    # Display the processed frame
    cv2.imshow("Processed Frame", new_frame)

    # Wait for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Read the next frame
    ret, frame = cap.read()

# Create video
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('Tareas/t2_output.avi', fourcc, 30, (1280, 720))

for i in range(len(new_video)):
    video.write(new_video[i])

# Release the video file and close windows
cap.release()
video.release()  
cv2.destroyAllWindows()
```
