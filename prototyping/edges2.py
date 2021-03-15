import cv2
import os
import glob
import numpy as np

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
]
FRAMESDIR = datasets[3]
PATH = os.path.dirname(os.path.realpath(__file__))

# load and sort frames from data directory
frames = glob.glob(PATH+'/AE4317_2019_datasets/'+FRAMESDIR+'/*.jpg')
frames.sort()
frames = [cv2.rotate(cv2.imread(img), cv2.ROTATE_90_COUNTERCLOCKWISE)  for img in frames]

# global variables
frame_current = frames[0]

def process_frame(frame):
    # get tunung controls
    blur_size = 2 * cv2.getTrackbarPos('blur_size', 'window') + 1
    canny_threshold1 = cv2.getTrackbarPos('canny_threshold1', 'window')
    canny_threshold2 = cv2.getTrackbarPos('canny_threshold2', 'window')

    frame_og = frame.copy()
    frame_blur = cv2.medianBlur(frame_og, blur_size)
    frame_gray = cv2.cvtColor(frame_og, cv2.COLOR_BGR2GRAY)
    # frame_gray_blur = cv2.GaussianBlur(frame_gray, (blur_size, blur_size), 0)
    # frame_gray_blur = cv2.medianBlur(frame_gray, blur_size)
    h, w = frame_gray.shape[:2]

    # Edges: Canny
    edges = cv2.Canny(frame_gray, canny_threshold1, canny_threshold2)

    dialate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges_dialated = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, dialate_kernel)
    # edges_dialated = cv2.dilate(edges, dialate_kernel)

    # Show forward path box
    fw_path_size = (100, int(h/3))
    forward_path_overlay = frame_og.copy()
    cv2.rectangle(forward_path_overlay, (int(w/2 - fw_path_size[0]/2), int(h/2 - fw_path_size[1]/2)), (int(w/2 + fw_path_size[0]/2), int(h/2 + fw_path_size[1]/2)), (0, 0, 255), thickness=-1)
    frame_og = cv2.addWeighted(forward_path_overlay, 0.5, frame_og, 0.5, 0)

    # Contours
    _, contours, _ = cv2.findContours(edges_dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame_og, contours, -1, (0, 255, 0), 1)

    boundRects = []
    for contour in contours:
        boundRect = cv2.boundingRect(contour)
        boundRects.append(boundRect)
        cv2.rectangle(frame_og, (int(boundRect[0]), int(boundRect[1])), (int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), (0, 0, 255), 1)

    

    # Show
    imshow_array = [
        frame_og,
        np.full((h, 1, 3), [0, 0, 255], np.uint8),
        cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR),
        np.full((h, 1, 3), [0, 0, 255], np.uint8),
        cv2.cvtColor(edges_dialated, cv2.COLOR_GRAY2BGR),
    ]
    cv2.imshow('window', np.hstack(imshow_array))

# Init loop
i = 0

# Create tuning controls
cv2.namedWindow('window')
cv2.createTrackbar('frame', 'window', 0, len(frames), lambda x: process_frame(frames[x]))
cv2.createTrackbar('blur_size', 'window', 3, 20, lambda x: process_frame(frames[i]))
cv2.createTrackbar('canny_threshold1', 'window', 200, 255, lambda x: process_frame(frames[i]))
cv2.createTrackbar('canny_threshold2', 'window', 100, 255, lambda x: process_frame(frames[i]))

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



 