import cv2
import os
import glob
import numpy as np
import random as rng

# dataset directory
datasets = [
    'cyberzoo_aggressive_flight/20190121-144646', # 0
    'cyberzoo_canvas_approach/20190121-151448',   # 1
    'cyberzoo_poles/20190121-135009',             # 2
    'cyberzoo_poles_panels/20190121-140205',      # 3
    'cyberzoo_poles_panels_mats/20190121-142935', # 4
    'sim_poles/20190121-160844',                  # 5
    'sim_poles_panels/20190121-161422',           # 6
    'sim_poles_panels_mats/20190121-161931',      # 7
    'flight_25032021',                            # 8
    'Exposure_30',                                # 9
    'Exposure_25',                                # 10
    'Exposure_20',                                # 11
    'Exposure_15',                                # 12
    'Exposure_10',                                # 13
    'Exposure_5',                                 # 14
]
FRAMESDIR = datasets[6]
PATH = os.path.dirname(os.path.realpath(__file__))

# Define default settings
""" Real, dataset """
# hor_thresh = 0.6
# blur_size = 25
# canny_thresh_1 = 200
# canny_thresh_2 = 100
# size_thresh = 75
# diff_thresh = 350 # python
# # diff_thresh = 100 # c++
""" Real, testflight """
# hor_thresh = 0.6
# blur_size = 25
# canny_thresh_1 = 200
# canny_thresh_2 = 100
# size_thresh = 75
# diff_thresh = 350 # python
# # diff_thresh = 100 # c++
""" Real, Exp30 """
# hor_thresh = 0.6
# blur_size = 21
# canny_thresh_1 = 255
# canny_thresh_2 = 150
# size_thresh = 75
# diff_thresh = 25
""" Sim """
hor_thresh = 0.6
blur_size = 41
canny_thresh_1 = 200
canny_thresh_2 = 250
size_thresh = 75
diff_thresh = 25

# load and sort frames from data directory
frames = glob.glob(PATH+'/AE4317_2019_datasets/'+FRAMESDIR+'/*.jpg')
frames.sort()
frames = [cv2.rotate(cv2.imread(img), cv2.ROTATE_90_COUNTERCLOCKWISE)  for img in frames]

# global variables
global i
i = 0

def process_frame(frame):
    
    
    # Get tunung controls
    blur_size = 2 * cv2.getTrackbarPos('Edge - blur_size', 'window') + 1
    canny_threshold1 = cv2.getTrackbarPos('Edge - canny_threshold1', 'window')
    canny_threshold2 = cv2.getTrackbarPos('Edge - canny_threshold2', 'window')
    boundRect_size_threshold = cv2.getTrackbarPos('BoundBox - size_threshold', 'window')
    boundRect_texture_threshold = cv2.getTrackbarPos('BoundBox - texture_threshold', 'window')
    hor_thr = cv2.getTrackbarPos('horizontal_threshold', 'window')
    
    # Setup frames
    #h, w = frame.shape[:2]
    #frame = cv2.resize(frame, (int(w/2), int(h/2)))
    h, w = frame.shape[:2]
    frame_og = frame.copy()
    frame_og2 = frame.copy()
    frame_blur = cv2.medianBlur(frame_og, blur_size)
    frame_gray = cv2.cvtColor(frame_og, cv2.COLOR_BGR2GRAY)
    # frame_gray_blur = cv2.GaussianBlur(frame_gray, (blur_size, blur_size), 0)
    # frame_gray_blur = cv2.medianBlur(frame_gray, blur_size)

    # Edges: Canny
    edges = cv2.Canny(frame_blur, canny_threshold1, canny_threshold2)
    # dialate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    # edges_dialated = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, dialate_kernel)

    # Show forward path box
    # fw_path_width = 75
    # forward_path_overlay = frame_og.copy()
    # cv2.rectangle(forward_path_overlay, (int(w/2 - fw_path_width/2), 0), (int(w/2 + fw_path_width/2), h), (0, 0, 255), thickness=-1)
    # frame_og = cv2.addWeighted(forward_path_overlay, 0.5, frame_og, 0.5, 0)

    # Show horizontal threshold
    hor_thr = int((hor_thr/100)*h)
    cv2.line(frame_og, (0, hor_thr), (w, hor_thr), (155, 0, 0), thickness=2)

    # Get contours
    # _, contours, _ = cv2.findContours(edges_dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Get bounding boxes
    frame_contours = np.zeros((h, w, 3), dtype=np.uint8)
    boundRects = []
    for j, contour in enumerate(contours):
        # get bounding box
        boundRect = cv2.boundingRect(contour)
        boundRects.append(boundRect)
        
        # get size
        boundRect_x = boundRect[0]
        boundRect_y = boundRect[1]
        boundRect_w = boundRect[2]
        boundRect_h = boundRect[3]
        boundRect_area = boundRect_w * boundRect_h

        #  show box
        cv2.rectangle(frame_og, (int(boundRect[0]), int(boundRect[1])), (int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), (255, 0, 0), 1)

        # get texture
        boundRect_img = frame_gray[int(boundRect[1]):int(boundRect[1]+boundRect[3]), int(boundRect[0]):int(boundRect[0]+boundRect[2])]
        avg = np.sum(boundRect_img)/boundRect_area
        diff = np.sum((boundRect_img - avg)**2)/boundRect_area
        diff = np.sum(np.abs(boundRect_img - avg))/boundRect_area

        # show texture
        text_color = (0, 0, 255)
        if (diff < boundRect_texture_threshold):
            text_color = (255, 0, 0)
        text_size = cv2.getTextSize(str(diff), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        text_size
        text_center = (
            # int(boundRect_x + boundRect_w/2 - text_size[0][0]/2),
            # int(boundRect_y + boundRect_h/2 - text_size[0][1]/2),
            int(boundRect_x + boundRect_w/2),
            int(boundRect_y + boundRect_h/2),
        )
        cv2.putText(frame_og, str(round(diff)), text_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color)

        # if (boundRect_w > int(w*0.02)) or (boundRect_h > int(h*0.02)):

        # boundRect_scale = 0.1
        # boundRect_img = frame_gray[
        #     int(boundRect[1]) : int(boundRect[1]+boundRect[3]), # y
        #     # int(boundRect[0]+boundRect[2]*boundRect_scale/2) : int(boundRect[0]+boundRect[2]*(1-boundRect_scale/2))  # x
        #     int(boundRect[0]+boundRect[2]*boundRect_scale/2) : int(boundRect[0]+boundRect[2]*(1-boundRect_scale/2))  # x
        # ]
        # avg = np.sum(boundRect_img)/(boundRect_w*(1-boundRect_scale) * boundRect_h)
        # diff = np.sum((boundRect_img - avg)**2)/(boundRect_w*(1-boundRect_scale) * boundRect_h)
        # cv2.rectangle(frame_og, (int(boundRect[0]+boundRect[2]*boundRect_scale/2), int(boundRect[1])), (int(boundRect[0]+boundRect[2]*(1-boundRect_scale/2)), int(boundRect[1]+boundRect[3])), (0, 255, 0), 1)

        # show bounding boxes + img 
        imshow_array = [
            cv2.cvtColor(boundRect_img, cv2.COLOR_GRAY2BGR),
        ]
        # cv2.imshow('box '+str(j), np.hstack(imshow_array))
        # if (boundRect_area >= (boundRect_size_threshold/10000)*(w*h)):
        #     cv2.imshow('box '+str(j), np.hstack(imshow_array))
        print('box '+str(j)+' diff='+str(round(diff)))

        # filter and show obstacles        
        color = (255, 0, 0)
        if ((boundRect_y+boundRect_h/2) < hor_thr) and (boundRect_area >= (boundRect_size_threshold/10000)*(w*h)) and (diff < boundRect_texture_threshold):
            color = (0, 0, 255)
            cv2.rectangle(frame_og2, (int(boundRect[0]), int(boundRect[1])), (int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), color, 2)

    # Show
    imshow_array = [
        frame_og2,
        np.full((h, 1, 3), [0, 0, 255], np.uint8),
        frame_og,
        np.full((h, 1, 3), [0, 0, 255], np.uint8),
        cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR),
        # cv2.cvtColor(edges_dialated, cv2.COLOR_GRAY2BGR),
    ]
    cv2.imshow('window', np.hstack(imshow_array))

# Create tuning controls
def trackbar_cb(x, frames):
    global i
    process_frame(frames[x])
    i = x

cv2.namedWindow('window')
cv2.createTrackbar('frame', 'window', 0, len(frames), lambda x: trackbar_cb(x, frames))
cv2.createTrackbar('Edge - blur_size', 'window', blur_size//2, 20, lambda x: trackbar_cb(i, frames))
cv2.createTrackbar('Edge - canny_threshold1', 'window', canny_thresh_1, 255, lambda x: trackbar_cb(i, frames))
cv2.createTrackbar('Edge - canny_threshold2', 'window', canny_thresh_2, 255, lambda x: trackbar_cb(i, frames))
cv2.createTrackbar('BoundBox - size_threshold', 'window', size_thresh, 100, lambda x: trackbar_cb(i, frames))
cv2.createTrackbar('BoundBox - texture_threshold', 'window', diff_thresh, 1000, lambda x: trackbar_cb(i, frames))
cv2.createTrackbar('horizontal_threshold', 'window', int(hor_thresh*100), 100, lambda x: trackbar_cb(i, frames))

# Loop
while True:
    process_frame(frames[i])
    k = cv2.waitKeyEx(0)
    if k == 32: i += 1 # space
    if k == 65513: i -= 1 # lalt
    if k == 100: i += 1 # D
    if k == 97: i -= 1 # A
    if k == 65362: i += 1 # up arrow
    if k == 65363: i += 1 # right arrow
    if k == 65361: i -= 1 # left arrow
    if k == 65364: i -= 1 # down arrow
    if k == 27: break # esc

cv2.destroyAllWindows()



 