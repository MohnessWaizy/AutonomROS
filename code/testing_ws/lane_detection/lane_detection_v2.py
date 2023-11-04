# Code from https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/

import cv2
import numpy as np
import matplotlib.pyplot as plt


def threshold_yellow(image):
    mask = cv2.inRange(image, (10, 70, 95), (100, 255, 255))
    return mask

def threshold_white(image):
    mask = cv2.inRange(image, (0,0,215), (179,35,255))
    return mask


def bird_view(roi_points, desired_roi_points, original_frame):
    transformation_matrix = cv2.getPerspectiveTransform(roi_points, desired_roi_points)
    warped_frame = cv2.warpPerspective(original_frame, transformation_matrix, (640, 480), flags=(cv2.INTER_LINEAR))
    #  warped_plot = cv2.polylines(warped_copy, desired_roi_points, True, (147, 20, 255), 3)
    return warped_frame


def bird_view_back(roi_points, desired_roi_points, frame):
    inv_transformation_matrix = cv2.getPerspectiveTransform(desired_roi_points, roi_points)
    warped_frame = cv2.warpPerspective(frame, inv_transformation_matrix, (640, 480), flags=(cv2.INTER_LINEAR))
    return warped_frame

def bird_view_back_points(roi_points, desired_roi_points, points):
    corrected_points = []
    M = cv2.getPerspectiveTransform(desired_roi_points, roi_points)
    for point in points:
        x = point[1]
        y = point[0]
        # https://stackoverflow.com/questions/55656057/how-to-use-cv2-warpperspective-with-only-one-source-point-x-y
        d = M[2, 0] * x + M[2, 1] * y + M[2, 2]
        corrected_points.append((
                int((M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d), # x
                int((M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d), # y
            )
        )
    return corrected_points


def calculate_histogram(frame=None):
    """
    Calculate the image histogram to find peaks in white pixel count

    :param frame: The warped image
    :param plot: Create a plot if True
    """

    # Generate the histogram
    histogram = np.sum(frame[:,:], axis=0)

    if False:

      # Draw both the image and the histogram
      figure, (ax1, ax2) = plt.subplots(2,1) # 2 row, 1 columns
      figure.set_size_inches(10, 5)
      ax1.imshow(frame, cmap='gray')
      ax1.set_title("Warped Binary Frame")
      ax2.plot(histogram)
      ax2.set_title("Histogram Peaks")
      plt.show()

    return histogram

def get_lane_line_indices_sliding_windows(histogram, warped_frame):
    """
    Get the indices of the lane line pixels using the 
    sliding windows technique.
         
    :param: plot Show plot or not
    :return: Best fit lines for the left and right lines of the current lane 
    """
    # Sliding window width is +/- margin
    margin = int((1/12) * 640)
 
    frame_sliding_window = warped_frame.copy()
 
    # Set the height of the sliding windows
    window_height = np.int(warped_frame.shape[0]/10)       
 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame. 
    nonzero = warped_frame.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1]) 
         
    # Store the pixel indices for the left and right lane lines
    left_lane_inds = []
         
    # Current positions for pixel indices for each window,
    # which we will continue to update
    leftx_base = np.argmax(histogram[:])
    leftx_current = leftx_base
 
    # Go through one window at a time
    no_of_windows = 10
         
    for window in range(no_of_windows):
       
        # Identify window boundaries in x and y (and right and left)
        win_y_low = 0 # warped_frame.shape[0] - (window + 1) * window_height
        win_y_high = 480 # warped_frame.shape[0] - window * window_height
        win_xleft_low = 0 # leftx_current - margin
        win_xleft_high = 640 #leftx_current + margin
        cv2.rectangle(frame_sliding_window,(win_xleft_low,win_y_low),(
        win_xleft_high,win_y_high), (255,255,255), 2)
 
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (
                           nonzerox < win_xleft_high)).nonzero()[0]
                                                         
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
         
        # If you found > minpix pixels, recenter next window on mean position
        minpix = int((1/24) * 640)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
                     
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
 
    # Extract the pixel coordinates for the left and right lane lines
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]

    weights = []
    for x, y in zip(leftx, lefty):
        weights.append(y ** 8)
 
    # Fit a second order polynomial curve to the pixel coordinates for
    # the left and right lane lines
    left_fit = np.polyfit(lefty, leftx, 2, w=weights) 
 
        
    # Create the x and y values to plot on the image  
    ploty = np.linspace(
        0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    # Generate an image to visualize the result
    out_img = np.dstack((
        frame_sliding_window, frame_sliding_window, (
        frame_sliding_window))) * 255

    # Add color to the left line pixels and right line pixels
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]

    # Plot the figure with the sliding windows
    if False:
        figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
        figure.set_size_inches(10, 10)
        figure.tight_layout(pad=3.0)
        ax1.imshow(cv2.cvtColor(warped_frame, cv2.COLOR_BGR2RGB))
        ax2.imshow(frame_sliding_window, cmap='gray')
        ax3.imshow(out_img)
        ax3.plot(left_fitx, ploty, color='yellow')
        ax1.set_title("Original Frame")  
        ax2.set_title("Warped Frame with Sliding Windows")
        ax3.set_title("Detected Lane Lines with Sliding Windows")
        plt.show()        

    return left_fit


def draw_tangent(frame, x, y, tangent):
    number_of_points = 30
    for i in range(number_of_points):
        x_s = i * (480 / number_of_points) - x
        y_s = tangent(x_s)

        x_cv = int(y + y_s)
        y_cv = int(x + x_s)
        cv2.circle(frame, (x_cv, y_cv), 4, 100, -1)


def calculate_trajectory_points(coefficients):
    points = []
    poly = lambda x: coefficients[0] * x ** 2 + coefficients[1] * x + coefficients[2]
    poly_slope = lambda x: 2 * coefficients[0] * x + coefficients[1]
    number_of_points = 30
    pixels_into_line = 150
    for i in range(number_of_points):
        x = i * (480 / number_of_points)
        y = poly(x)
        slope = poly_slope(x)
        tangent_90_deg = lambda x: - (1 / slope) * x

        x_s = - (slope * pixels_into_line) / ((slope ** 2 + 1) ** (1 / 2))
        y_s = tangent_90_deg(x_s)
        points.append((x + x_s, y + y_s))
    return points


def main():
    original_frame = cv2.imread('images/inrightcurve.png')
    original_frame = cv2.cvtColor(original_frame, cv2.COLOR_BGR2HSV)
    threshold_frame = threshold_yellow(original_frame)
    # threshold_frame = threshold_white(original_frame)
    # cv2.imshow("Mask", original_frame)
    # cv2.waitKey(0)
    # white_lane = threshold_white(original_frame)
    # original_frame = cv2.bitwise_or(yellow_lane, white_lane)

    # bottom left, bottom right, upper left, upper right
    roi_points = np.float32([(0, 345), (564, 345), (247, 263), (375, 263)])
    padding = int(0.25 * 640)
    desired_roi_points = np.float32(
        [
            (padding, 480),
            (640-padding, 480),
            (padding, 0),
            (640-padding, 0)
        ]
    )
    threshold_frame = bird_view(roi_points, desired_roi_points, threshold_frame)
    histogram = calculate_histogram(threshold_frame)
    coefficients = get_lane_line_indices_sliding_windows(histogram, threshold_frame)
    points = calculate_trajectory_points(coefficients)
    corrected_points = bird_view_back_points(roi_points, desired_roi_points, points)
    frame = cv2.cvtColor(original_frame, cv2.COLOR_HSV2BGR)
    for point in corrected_points:
        cv2.circle(frame, (int(point[0]), int(point[1])), 4, (0, 98, 0), -1)

    cv2.imshow("Mask", frame)
    #cv2.imshow("Mask", threshold_frame)
    cv2.waitKey(0)
    # poly = lambda x: coefficients[0] * x ** 2 + coefficients[1] * x + coefficients[2]
    # slope = lambda x: 2 * coefficients[0] * x + coefficients[1]
    # number_of_points = 30
    # pixels_into_line = 50000
    # for i in range(number_of_points):
    #     x = i * (480 / number_of_points)
    #     y = poly(x)
    #     y_slope = - 1 / slope(x)
    #     way_into_line = (pixels_into_line ** (1 / 2)) / ((y_slope ** 2 + 1) ** (1 / 2))
    #     cv2.circle(original_frame, (int(y + way_into_line), int(x + y_slope * way_into_line)), 4, 100, -1)
    # cv2.imshow("Mask", original_frame)
    # cv2.waitKey(0)



main()
