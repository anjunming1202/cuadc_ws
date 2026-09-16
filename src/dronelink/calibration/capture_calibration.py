import cv2
import os
import time

sdp_path = os.path.expandvars(r"%USERPROFILE%\Desktop\DroneLink.sdp")
output_dir = os.path.expandvars(r"%USERPROFILE%\Desktop\calibration_images")
os.makedirs(output_dir, exist_ok=True)

SAVE_INTERVAL = 1.0

cap = cv2.VideoCapture(sdp_path, cv2.CAP_FFMPEG)
if not cap.isOpened():
    print("ERROR: Cannot open DroneLink video.")
    input("Press Enter to exit...")
    raise SystemExit

print("DroneLink calibration capture started.")
print("Saving one image every second.")
print("Press Q in the video window to stop.")

last_save = 0
saved_count = 0

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    now = time.time()
    if now - last_save >= SAVE_INTERVAL:
        last_save = now
        saved_count += 1
        filename = os.path.join(output_dir, f"calib_{saved_count:04d}.jpg")
        cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f"Saved: {filename}")

    preview = cv2.resize(frame, (960, 540))
    cv2.putText(preview, f"Saved: {saved_count} | 1 image/sec | Q = quit",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow("DroneLink Calibration Capture", preview)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print(f"Finished. Saved {saved_count} images.")
