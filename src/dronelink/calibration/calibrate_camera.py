import cv2
import numpy as np
import glob
import os

BOARD_SIZE = (9, 6)
SQUARE_SIZE = 24.0  # mm

image_dir = os.path.expandvars(r"%USERPROFILE%\Desktop\calibration_images")
images = sorted(glob.glob(os.path.join(image_dir, "*.jpg")))
print(f"Found {len(images)} images.")

objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []
image_size = None
successful_images = []

for i, filename in enumerate(images, 1):
    image = cv2.imread(filename)
    if image is None:
        print(f"[{i}/{len(images)}] READ ERROR")
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]
    found, corners = cv2.findChessboardCornersSB(
        gray, BOARD_SIZE, flags=cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if found:
        objpoints.append(objp.copy())
        imgpoints.append(corners)
        successful_images.append(filename)
        print(f"[{i}/{len(images)}] FOUND")
    else:
        print(f"[{i}/{len(images)}] NOT FOUND")

print()
print(f"Valid calibration images: {len(successful_images)}")
if len(successful_images) < 10:
    print("ERROR: Too few valid checkerboard images.")
    input("Press Enter to exit...")
    raise SystemExit

rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, image_size, None, None
)

print("\n========== CALIBRATION RESULT ==========")
print("\nImage size:")
print(image_size)
print("\nRMS reprojection error:")
print(rms)
print("\nCamera matrix:")
print(camera_matrix)
print("\nDistortion coefficients:")
print(dist_coeffs)

errors = []
for i in range(len(objpoints)):
    projected, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
    )
    error = cv2.norm(
        imgpoints[i].reshape(-1, 2),
        projected.reshape(-1, 2),
        cv2.NORM_L2,
    ) / len(projected)
    errors.append(error)

print("\nMean per-image reprojection error:")
print(np.mean(errors))

output_file = os.path.expandvars(r"%USERPROFILE%\Desktop\camera_calibration.npz")
np.savez(
    output_file,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    image_size=image_size,
    rms=rms,
)

print("\nCalibration saved to:")
print(output_file)
print("\n========================================")
input("Press Enter to exit...")
