# --------------------------------------------------------------------------
# Helper: utils/check_camera_ids.py (figure out camera indices to use for capture_stereo.py)
# --------------------------------------------------------------------------
import cv2

def check_camera_indices(max_to_test: int = 10) -> None:
    found_indices = []
    
    print(f"Scanning indices 0 through {max_to_test-1}...")
    print("Press 'q' or 'ESC' to close the test windows.")

    # We test a range of indices
    for i in range(max_to_test):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        
        if cap.isOpened():
            # Try to read a frame to confirm it's actually working
            ret, frame = cap.read()
            if ret:
                found_indices.append(i)
                cv2.putText(frame, f"INDEX: {i}", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow(f"Camera Index {i}", frame)
                print(f" [+] Success: Camera found at index {i}")
            cap.release()
        else:
            pass

    if not found_indices:
        print(" [!] No cameras were detected. Check your USB connections.")
    else:
        print(f"\nScan complete. Found {len(found_indices)} cameras at indices: {found_indices}")
        print("Identify your 'Left' and 'Right' cameras by looking at the windows.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    check_camera_indices()