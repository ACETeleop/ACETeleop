import cv2
import time


def list_available_cameras(max_tested=10):
    index = 0
    arr = []
    while index < max_tested:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
        cap.release()
        index += 1
    return arr


# Retrieve the list of available cameras
available_cameras = list_available_cameras()

if not available_cameras:
    print("Error: No cameras found.")
    exit()

# Initialize the first available camera
current_index = 0
cap = cv2.VideoCapture(available_cameras[current_index])
window_name = f"Webcam {available_cameras[current_index]}"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

Time = time.perf_counter()
try:
    while True:
        tac = time.perf_counter()
        print(1 / (tac - Time))
        Time = tac

        ret, frame = cap.read()
        if ret:
            # Display the frame using OpenCV
            cv2.imshow(window_name, frame)
        else:
            print(
                f"Error: Could not read frame from camera {available_cameras[current_index]}."
            )
            break

        key = cv2.waitKey(1)
        if key & 0xFF == ord("q"):
            break
        elif key & 0xFF == ord("k"):
            # Close the current camera and move to the next
            cap.release()
            cv2.destroyWindow(window_name)
            current_index += 1
            if current_index >= len(available_cameras):
                print("No more cameras available.")
                break
            cap = cv2.VideoCapture(available_cameras[current_index])
            window_name = f"Webcam {available_cameras[current_index]}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
finally:
    # When everything done, release the capture and destroy all windows
    cap.release()
    cv2.destroyAllWindows()
