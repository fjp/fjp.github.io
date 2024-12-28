---
layout: single
title:  "Camera calibration and reprojection error"
date:   2024-12-27 17:31:41 +0200
excerpt: "Camera calibration and reprojection error."
permalink: /posts/computer-vision/camera-calibration-reprojection-error/
categories: [robotics, computer vision, algorithms]
tags: [robotics, computer vision, python, opencv, matplotlib]
comments: true
use_math: true
toc: true
classes: wide
# toc_label: "Unscented Kalman Filter"
header:
  teaser: /assets/images/opencv_logo.png
  overlay_image: /assets/images/opencv_logo.png #keep it square 200x200 px is good
---



# Camera calibration and reprojection error

Imagine you're taking a photo of a 3D object, like a cube. When you take the picture, you're converting 3D points in the real world into 2D points on your photo (this is projection). Now, if you know the camera's properties (like position, orientation, focal length, etc.), you can mathematically predict where each 3D point should appear in the 2D image.

The reprojection error is the difference between:

1. Where you actually observe a point in your 2D image
2. Where your mathematical model predicts that point should appear

Think of it as measuring how accurate your camera model is. A smaller reprojection error means your camera model better matches reality. It's typically measured in pixels - if your model predicts a point should be at pixel coordinates (100, 100) but you actually observe it at (102, 101), your reprojection error would be about 2.2 pixels.

Since you work with Python, here's a simplified example of how reprojection error might be calculated:

```python
def calculate_reprojection_error(observed_points, projected_points):
    """
    Calculate reprojection error between observed and projected points
    
    Args:
        observed_points: Nx2 array of observed 2D points in image
        projected_points: Nx2 array of predicted 2D points from 3D->2D projection
    
    Returns:
        Mean reprojection error in pixels
    """
    import numpy as np
    
    # Calculate Euclidean distance between each pair of points
    errors = np.sqrt(np.sum((observed_points - projected_points) ** 2, axis=1))
    
    # Return mean error
    return np.mean(errors)
```

Let's say we have:

A 3D point in real world: P = (X=100mm, Y=50mm, Z=1000mm)
A camera with focal length f = 800 pixels
Camera center (principal point) at image coordinates (cx=320, cy=240)

The basic pinhole camera projection equation is:

```python
# Convert 3D world point to 2D image point
x = f * (X/Z) + cx
y = f * (Y/Z) + cy
```

Let's plug in our numbers:

```python
# Predicted image coordinates from our model
x = 800 * (100/1000) + 320 = 400 pixels
y = 800 * (50/1000) + 240 = 280 pixels

# So our model predicts point P will appear at (400, 280) in the image
predicted = (400, 280)

# But when we actually look at our image, we find the point at
actual = (405, 283)

# Reprojection error = sqrt((405-400)² + (283-280)²)
error = sqrt(25 + 9) = sqrt(34) ≈ 5.8 pixels
```

This difference occurs because:

Real cameras aren't perfect pinhole cameras
There might be lens distortion
Our camera parameters (focal length, center) might not be exactly right
Measurement errors when detecting points in the image

This is why we use reprojection error during camera calibration - we try to find camera parameters that minimize this error across many points.

## Calibration process

A chessboard pattern is used for camera calibration because it provides several key advantages:

1. Easy point detection - The strong black and white contrast of corners makes them very reliable to detect automatically in images. These corners are what we call the "control points" - points whose positions we know very precisely in both 3D space and the 2D image.
2. Known geometry - Since a chessboard is flat (planar) and has equal square sizes, we know the exact physical distances between all corner points. For example, if each square is 30mm, we know that diagonal corners of a square are exactly 42.4mm apart (30 * √2). This gives us ground truth 3D coordinates.

Here's how it works:

```python
# Let's say our chessboard squares are 30mm
# For a 9x6 board, we can define the 3D coordinates of every corner as:
corners_3d = []
for y in range(6):
    for x in range(9):
        # Z=0 because board is flat (in XY plane)
        corners_3d.append([x*30, y*30, 0])  

# When we take a photo, our corner detector finds the corresponding 
# 2D pixel coordinates of these same points in the image
corners_2d = detect_chessboard_corners(image)  # Returns pixel coordinates

# Now we can solve for camera parameters by minimizing the reprojection error
# between these known 3D->2D point correspondences
```

Each photo of the chessboard from different angles gives us more point correspondences, improving our calibration accuracy. The chessboard essentially gives us a very precise ruler in 3D space that we can use to figure out exactly how our camera maps 3D points to 2D image points.


### Using multiple views

The brilliant thing about using multiple chessboard views is that we don't need to know the actual distances from the camera or the exact 3D locations of the chessboard! Here's why:
When calibrating:

1. We take photos of the chessboard in different orientations (tilted, rotated, at different distances)
2. We arbitrarily set our world coordinate system where:
   - The chessboard lies in the XY plane (Z=0)
   - The top-left corner is at (0,0,0)
   - We only need to know the physical size of each square (e.g., 30mm)

For each view, the calibration algorithm solves for:

- The camera's intrinsic parameters (focal length, principal point, distortion)
- The extrinsic parameters (rotation and translation) that describe where the chessboard is relative to the camera for that specific view

```python
# For each view i:
#   R[i] = 3x3 rotation matrix
#   t[i] = 3x1 translation vector
#   These transform points from chessboard coordinates to camera coordinates

# A point P on the chessboard goes through these transformations:
# 1. Start with chessboard coordinates (we know these from square size)
P_chess = [x*30, y*30, 0]

# 2. Transform to camera coordinates for this view
P_cam = R[i] @ P_chess + t[i]

# 3. Project to image coordinates using camera intrinsics
x = f * (P_cam[0]/P_cam[2]) + cx
y = f * (P_cam[1]/P_cam[2]) + cy
```

Best practices for capturing views:

- Take 10-20 different views
- Vary the orientation (rotate board ~45° in different directions)
- Vary the distance (but keep corners clearly visible)
- Cover different parts of the image (especially corners)
- Keep the board relatively flat (don't shoot at extremely steep angles)

The algorithm uses all these views together to solve for the camera parameters that minimize the total reprojection error across all points in all views. Each view adds more constraints to the system, making the calibration more robust.


### Optimization process in detail

The following shows a practical example that illustrates both the optimization process and the implementation.

```python
import numpy as np
import cv2
from scipy.optimize import minimize
from dataclasses import dataclass

@dataclass
class CameraParams:
    """Camera intrinsic parameters"""
    fx: float  # focal length x
    fy: float  # focal length y
    cx: float  # principal point x
    cy: float  # principal point y
    k1: float  # radial distortion coefficient 1
    k2: float  # radial distortion coefficient 2

def create_chessboard_points(board_size, square_size):
    """Create 3D points for chessboard corners in board coordinate system"""
    points = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    points[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    return points * square_size

def project_points(points_3d, rvec, tvec, camera_params):
    """Project 3D points to 2D using camera parameters"""
    # Convert rotation vector to matrix
    R, _ = cv2.Rodrigues(rvec)
    
    # Transform points to camera coordinate system
    points_cam = (R @ points_3d.T + tvec.reshape(3, 1)).T
    
    # Perspective division
    x = points_cam[:, 0] / points_cam[:, 2]
    y = points_cam[:, 1] / points_cam[:, 2]
    
    # Apply radial distortion
    r2 = x*x + y*y
    distortion = (1 + camera_params.k1*r2 + camera_params.k2*r2*r2)
    x_distorted = x * distortion
    y_distorted = y * distortion
    
    # Project to image coordinates
    points_2d = np.column_stack([
        camera_params.fx * x_distorted + camera_params.cx,
        camera_params.fy * y_distorted + camera_params.cy
    ])
    
    return points_2d

def calibration_error(params, points_3d, views_2d):
    """Calculate total reprojection error across all views"""
    n_views = len(views_2d)
    n_points = len(points_3d)
    
    # Unpack optimization parameters
    camera_params = CameraParams(
        fx=params[0], fy=params[1],
        cx=params[2], cy=params[3],
        k1=params[4], k2=params[5]
    )
    
    # Rotation and translation for each view
    rvecs = params[6:6+n_views*3].reshape(n_views, 3)
    tvecs = params[6+n_views*3:].reshape(n_views, 3)
    
    total_error = 0
    
    # Calculate error for each view
    for i, points_2d in enumerate(views_2d):
        projected = project_points(points_3d, rvecs[i], tvecs[i], camera_params)
        error = np.sum((points_2d - projected) ** 2)
        total_error += error
        
    return total_error

def calibrate_camera(board_size, square_size, views_2d):
    """
    Perform camera calibration using multiple chessboard views
    
    Args:
        board_size: Tuple of (rows, cols) interior corners
        square_size: Physical size of squares in mm
        views_2d: List of detected corner coordinates for each view
    """
    # Create 3D points in board coordinate system
    points_3d = create_chessboard_points(board_size, square_size)
    
    n_views = len(views_2d)
    
    # Initial guess for camera parameters
    image_size = views_2d[0].max(axis=0)
    initial_params = np.array([
        image_size[0],  # fx
        image_size[0],  # fy
        image_size[0]/2,  # cx
        image_size[1]/2,  # cy
        0.0,  # k1
        0.0,  # k2
    ])
    
    # Initial guess for pose of each view
    initial_rvecs = np.zeros(n_views * 3)
    initial_tvecs = np.zeros(n_views * 3)
    initial_tvecs[2::3] = -500  # Start with boards 500mm away
    
    # Combine all parameters for optimization
    x0 = np.concatenate([initial_params, initial_rvecs, initial_tvecs])
    
    # Run optimization
    result = minimize(
        calibration_error,
        x0,
        args=(points_3d, views_2d),
        method='Nelder-Mead',
        options={'maxiter': 1000}
    )
    
    # Extract optimized parameters
    camera_params = CameraParams(
        fx=result.x[0], fy=result.x[1],
        cx=result.x[2], cy=result.x[3],
        k1=result.x[4], k2=result.x[5]
    )
    
    rvecs = result.x[6:6+n_views*3].reshape(n_views, 3)
    tvecs = result.x[6+n_views*3:].reshape(n_views, 3)
    
    return camera_params, rvecs, tvecs

# Example usage:
def main():
    # Define chessboard properties
    board_size = (9, 6)  # interior corners
    square_size = 30  # mm
    
    # Load and detect corners in images
    images = [cv2.imread(f'calib_image_{i}.jpg') for i in range(10)]
    views_2d = []
    
    for img in images:
        ret, corners = cv2.findChessboardCorners(img, board_size)
        if ret:
            # Refine corner detection
            corners = cv2.cornerSubPix(
                cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                corners,
                (11, 11),
                (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            views_2d.append(corners.reshape(-1, 2))
    
    # Perform calibration
    camera_params, rvecs, tvecs = calibrate_camera(board_size, square_size, views_2d)
    
    print("Calibrated camera parameters:")
    print(f"Focal length: ({camera_params.fx}, {camera_params.fy})")
    print(f"Principal point: ({camera_params.cx}, {camera_params.cy})")
    print(f"Distortion coefficients: k1={camera_params.k1}, k2={camera_params.k2}")
```

the optimization process works:

1. Initial Guess:

- We start with reasonable guesses for camera parameters (focal length ≈ image width, principal point ≈ image center)
- We assume each chessboard starts roughly facing the camera and 500mm away


2. Optimization Loop:

- For each set of parameters, we:
    a. Project all 3D chessboard points to 2D using current camera parameters
    b. Calculate total reprojection error (sum of squared distances between projected and detected points)
    c. The optimizer (Nelder-Mead in this case) adjusts parameters to minimize this error
- This continues until the error converges or max iterations reached


3. What's being optimized:

- Camera intrinsics: focal length (fx, fy), principal point (cx, cy), distortion (k1, k2)
- For each view: rotation (3 parameters) and translation (3 parameters)
- Total parameters = 6 + 6 × number_of_views



The key points about distances:

- We don't need to know the actual camera-to-board distances
- The optimization finds the relative positions (tvecs) automatically
- The only physical measurement we need is the square size, which sets the scale

## Camera Calibration Quality Analysis

Here's what we're analyzing and what to look for:

1. Mean Reprojection Error:

- Under 0.5 pixels: Excellent
- Under 1.0 pixels: Good
- Under 2.0 pixels: Fair
- Over 2.0 pixels: Poor

2. Error Distribution:

- Should be roughly Gaussian
- No significant outliers
- Standard deviation should be small relative to mean

3. Spatial Distribution:

- Errors should be uniform across the image
- Watch for patterns or clusters of high error
- Higher errors at image edges might indicate lens distortion issues

4. Distance Effects:

- Errors shouldn't strongly correlate with board distance
- If they do, might indicate:
    - Scale issues
    - Focus problems
    - Insufficient distance variation in calibration images


### Red Flags to Watch For:

1. Individual views with much higher error than others
2. Strong patterns in the error heatmap
3. Sudden jumps in error with distance
4. Very different errors in x and y directions
5. Large outliers in any metric


## Application of calibration parameters

Once you have your calibrated camera parameters, you can use them for several key applications:

1. Undistort Images:

```python
# Remove lens distortion from images
undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)
```

2. 3D to 2D Projection (World to Image coordinates):

```python
# Project 3D points to image plane
image_points, _ = cv2.projectPoints(object_points_3d, 
                                  rvec, tvec, 
                                  camera_matrix, 
                                  dist_coeffs)
```

3. 2D to 3D Ray (Image to World direction):

```python
# Convert image point to normalized ray
image_point = np.array([x, y])
normalized_point = cv2.undistortPoints(image_point, 
                                     camera_matrix,
                                     dist_coeffs)
# This gives you a ray direction in camera coordinates
```

4. Pose Estimation (Find object position):

```python
# Find position of an object with known 3D points
success, rvec, tvec = cv2.solvePnP(object_points_3d,
                                  detected_points_2d,
                                  camera_matrix,
                                  dist_coeffs)
```

5. Stereo Calibration (If you have two cameras):

```python
# Calibrate stereo pair using individual camera parameters
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = \
    cv2.stereoCalibrate(object_points,
                        imagePoints1,
                        imagePoints2,
                        camera_matrix1,
                        dist_coeffs1,
                        camera_matrix2,
                        dist_coeffs2,
                        image_size)
```

6. Save parameters for later use:

```python
# Save calibration results to file
calibration_data = {
    'camera_matrix': camera_matrix,
    'dist_coeffs': dist_coeffs,
    'image_size': image_size
}
np.save('camera_calibration.npy', calibration_data)

# Load later
loaded_calib = np.load('camera_calibration.npy', allow_pickle=True).item()
```

These parameters are essential for:

- 3D reconstruction
- Augmented reality applications
- Visual SLAM
- Object pose estimation
- Depth estimation in stereo systems
- Any application requiring accurate mapping between 3D world and 2D image

