import cv2
import numpy as np
import matplotlib.pyplot as plt
import os.path

def intersection(p1,p2,p3,p4):
    slope_a = (p2[1] - p1[1]) / (p2[0] - p1[0])
    slope_b = (p4[1] - p3[1]) / (p4[0] - p3[0])
    inter_a = p1[1] - slope_a * p1[0]
    inter_b = p3[1] - slope_b * p3[0]
    x = (inter_b - inter_a) / (slope_a - slope_b)
    y = (slope_a * x) + inter_a
    return (int(x),int(y))

def make_coordinates(line_parameters):
    slope = line_parameters[0]
    intercept = line_parameters[1]
    #make coordinates
    y1 = int(height)
    y2 = int(y1*(2/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return [(x1,y1),(x2,y2)]

def filter_lines(lines):
    left_line_fit = []
    right_line_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1,x2), (y1,y2), 1)
        print(parameters)
        print(type(parameters))
        slope = parameters[0]
        print(slope)
        intercept = parameters[1]
        print(intercept)
        print(parameters)
        if slope > 0:
            right_line_fit.append((slope,intercept))
        if slope < 0 and intercept > height:
            left_line_fit.append((slope,intercept))
    if (len(right_line_fit) > 0) and (len(left_line_fit) >0):
        left_line_average = np.average(left_line_fit, axis=0)
        right_line_average = np.average(right_line_fit, axis=0)
        left_line = make_coordinates(left_line_average)
        right_line = make_coordinates(right_line_average)
        return (left_line,right_line)
    return (None,None)
    
    

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    #channel_count = img.shape[2]
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_the_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    if lines is not None: 
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(blank_image, (x1,y1), (x2,y2), (0, 255, 0), thickness=10)

        img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img


def timer_callback():
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """

    global error
    global image_input
    global just_seen_line
    global should_move
    global finalization_countdown
    global height
    global width

    # Wait for the first image to be received
    if type(image_input) != np.ndarray:
        return

    print(image_input.shape)
    height = image_input.shape[0]
    width = image_input.shape[1]

    image = image_input.copy()

    region_of_interest_vertices = [
        (int(0), int(3*height/4)),
        (int(width/2), int(height/4)),
        (int(width), int(3*height/4))
    ]
    
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    canny_image = cv2.Canny(gray_image, 100, 120)
    cropped_image = region_of_interest(canny_image,
                    np.array([region_of_interest_vertices], np.int32),)
    lines = cv2.HoughLinesP(cropped_image,
                            rho=2,
                            theta=np.pi/180,
                            threshold=100,
                            lines=np.array([]),
                            minLineLength=40,
                            maxLineGap=100)
        
    image_with_lines = draw_the_lines(image, lines)
    
    cv2.line(image_with_lines, region_of_interest_vertices[0],region_of_interest_vertices[1] ,(10,100,240),thickness=3)
    cv2.line(image_with_lines, region_of_interest_vertices[1],region_of_interest_vertices[2] ,(10,100,240),thickness=3)
    cv2.line(image_with_lines, region_of_interest_vertices[0],region_of_interest_vertices[2] ,(10,100,240),thickness=3)
    if lines is not None:
        left_line, right_line = filter_lines(lines)
        if ((left_line is not None) and (right_line is not None)):
            cv2.line(image_with_lines, left_line[0], left_line[1] ,(100,150,0),thickness=3)
            cv2.line(image_with_lines, right_line[0],right_line[1] ,(100,150,0),thickness=3)
      

            left_point = intersection(left_line[0], left_line[1],region_of_interest_vertices[0],region_of_interest_vertices[2])
            right_point = intersection(right_line[0],right_line[1],region_of_interest_vertices[0],region_of_interest_vertices[2])
            trajectory_point = (int((left_point[0]+right_point[0])/2), right_point[1])
            print("trajectory: ", trajectory_point)
            
            cv2.circle(image_with_lines, trajectory_point, radius=8, color=(0,0,255), thickness=-1)
            
    #     message.linear.x = LINEAR_SPEED
    #     just_seen_line = True

    #     # plot the line centroid on the image
    #     cv2.circle(output, (line['x'], crop_h_start + line['y']), 5, (0,255,0), 7)

    # else:
    #     # There is no line in the image. 
    #     # Turn on the spot to find it again. 
    #     if just_seen_line:
    #         just_seen_line = False
    #         error = error * LOSS_FACTOR
    #     message.linear.x = 0.0


    
    # # Determine the speed to turn and get the line in the center of the camera.
    # message.angular.z = float(error) * -KP
    # print("Error: {} | Angular Z: {}, ".format(error, message.angular.z))
    

    # Show the output image to the user
    cv2.imshow("output", image_with_lines)
    cv2.waitKey(0)

global image_input
# The video feed is read in as a VideoCapture object
print(os.getcwd())
file_exists = os.path.exists('src//autonomros_lane_detection//test//test.jpg')

print(file_exists)
image_input = cv2.imread('src//autonomros_lane_detection//test//test.jpg')
timer_callback()
cv2.destroyAllWindows()