import numpy as np
import cv2
import os




# Global vars. initial values
image_input = 0
error = 0
just_seen_line = False
should_move = False
height = 0
width = 0



hue_white_l = 0
hue_white_h = 179
saturation_white_l = 0
saturation_white_h = 35
lightness_white_l = 215
lightness_white_h = 255

lower_white = np.array([hue_white_l, saturation_white_l, lightness_white_l])
upper_white = np.array([hue_white_h, saturation_white_h, lightness_white_h])

reliability_white_line = 100



def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.

    return (0*height//20, 15*height//20, 0*width//8, 8*width//8)        

def get_line(image):
    """
    Tune the saturation and brightness of the image
    """
    global reliability_white_line
    global hue_white_l
    global hue_white_h
    global saturation_white_l
    global saturation_white_h
    global lightness_white_l
    global lightness_white_h
    global lower_white 
    global upper_white 

    global crop_w_start
    height, width, _ = image.shape
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    # get the bottom part of the image (matrix slicing)
    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    res = cv2.bitwise_and(crop, crop, mask = mask)

    res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    
    #Remove white dots from image 

    # convert to binary by thresholding
    ret, binary_map = cv2.threshold(res,127,255,0)

    # do connected components processing
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map)

    #get CC_STAT_AREA component as stats[label, COLUMN] 
    areas = stats[1:,cv2.CC_STAT_AREA]

    result = np.zeros((labels.shape), np.uint8)

    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255

    fraction_num = np.count_nonzero(mask)
    if fraction_num > 35000:
        if lightness_white_l < 250:
                lightness_white_l += 5
    elif fraction_num < 5000:
        if lightness_white_l > 50:
            lightness_white_l -= 5

    how_much_short = 0
    for i in range(0, 600):
        if np.count_nonzero(mask[i,::]) > 0:
            how_much_short += 1
    how_much_short = 600 - how_much_short

    if how_much_short > 100:
        if reliability_white_line >= 5:
            reliability_white_line -= 5
    elif how_much_short <= 100:
        if reliability_white_line <= 99:
            reliability_white_line += 5


    return result


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
        (int(0), int(height)),
        (int(width/2), int(height/4)),
        (int(width), int(height))
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

    #cv2.rectangle(image_with_lines, region_of_interest_vertices[0],region_of_interest_vertices[2], (20,130,2), thickness=3)


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
    #cv2.imshow("output", image_with_lines)
    return image_with_lines

    #cv2.waitKey(0)
    #return image_with_lines





def main():
    dir = 'images'
    for f in os.listdir(dir):
        os.remove(os.path.join(dir, f))

    image = cv2.imread(os.path.join(".","double_lane.jpg"))
    image_result = cv2.imread(os.path.join(".","image_preprocessing.jpg"))
    annotated_image = get_line(image)

    global image_input

    image_input = image_result 
    image_output = timer_callback()

    cv2.imwrite('image_preprocessing.jpg', annotated_image)
    cv2.imwrite('images/detected_lines.jpg',image_output)
    

    cv2.destroyAllWindows()

    #cv2.imwrite('images/detected_lines.jpg', image_with_lines)

main()