import cv2
import numpy as np
import os
import glob

BOARD_SIZE = (9, 6)
calib_file = os.path.expandvars(r"%USERPROFILE%\Desktop\camera_calibration.npz")
image_dir = os.path.expandvars(r"%USERPROFILE%\Desktop\calibration_images")
output_file = os.path.expandvars(r"%USERPROFILE%\Desktop\undistort_checkerboard_compare.jpg")

data = np.load(calib_file)
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

images = sorted(glob.glob(os.path.join(image_dir, "*.jpg")))
if not images:
    print("ERROR: No calibration images found.")
    input("Press Enter to exit...")
    raise SystemExit

filename = images[len(images) // 2]
image = cv2.imread(filename)
if image is None:
    print("ERROR: Cannot read image.")
    input("Press Enter to exit...")
    raise SystemExit

h, w = image.shape[:2]
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix, dist_coeffs, (w, h), 1, (w, h)
)
undistorted = cv2.undistort(
    image, camera_matrix, dist_coeffs, None, new_camera_matrix
)

def draw_checkerboard_lines(img):
    result = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray, BOARD_SIZE, flags=cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        return result, False

    pts = corners.reshape(BOARD_SIZE[1], BOARD_SIZE[0], 2)

    for row in range(BOARD_SIZE[1]):
        for col in range(BOARD_SIZE[0] - 1):
            p1 = tuple(np.round(pts[row, col]).astype(int))
            p2 = tuple(np.round(pts[row, col + 1]).astype(int))
            cv2.line(result, p1, p2, (0, 255, 0), 3)

    for col in range(BOARD_SIZE[0]):
        for row in range(BOARD_SIZE[1] - 1):
            p1 = tuple(np.round(pts[row, col]).astype(int))
            p2 = tuple(np.round(pts[row + 1, col]).astype(int))
            cv2.line(result, p1, p2, (0, 0, 255), 3)

    for point in corners.reshape(-1, 2):
        p = tuple(np.round(point).astype(int))
        cv2.circle(result, p, 6, (255, 0, 0), -1)

    return result, True

original_lines, original_found = draw_checkerboard_lines(image)
undistorted_lines, undistorted_found = draw_checkerboard_lines(undistorted)

preview_original = cv2.resize(original_lines, (960, 540))
preview_undistorted = cv2.resize(undistorted_lines, (960, 540))
comparison = np.hstack((preview_original, preview_undistorted))

cv2.putText(comparison, "ORIGINAL", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255, 255, 255), 2)
cv2.putText(comparison, "UNDISTORTED", (980, 40), cv2.FONT_HERSHEY_SIMPLEX,
            1, (255, 255, 255), 2)

cv2.imwrite(output_file, comparison)
print("Image:")
print(filename)
print("\nOriginal checkerboard detected:", original_found)
print("Undistorted checkerboard detected:", undistorted_found)
print("\nComparison saved to:")
print(output_file)
print("\nGREEN = horizontal checkerboard lines")
print("RED   = vertical checkerboard lines")
print("BLUE  = detected corners")
print("\nLeft  = original")
print("Right = undistorted")
print("\nPress any key in the image window to exit.")

cv2.imshow("Camera Calibration Check", comparison)
cv2.waitKey(0)
cv2.destroyAllWindows()
