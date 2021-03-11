import cv2
import os
import glob
import numpy as np

# dataset directory
datasets = [
    'cyberzoo_aggressive_flight/20190121-144646',
    'cyberzoo_canvas_approach/20190121-151448',
    'cyberzoo_poles/20190121-135009',
    'cyberzoo_poles_panels/20190121-140205',
    'cyberzoo_poles_panels_mats/20190121-142935',
    'sim_poles/20190121-160844',
    'sim_poles_panels/20190121-161422',
    'sim_poles_panels_mats/20190121-161931',
]
FRAMESDIR = datasets[3]
PATH = os.path.dirname(os.path.realpath(__file__))

# load and sort frames from data directory
frames = glob.glob(PATH+'/AE4317_2019_datasets/'+FRAMESDIR+'/*.jpg')
frames.sort()
frames = [cv2.rotate(cv2.imread(img), cv2.ROTATE_90_COUNTERCLOCKWISE)  for img in frames]

# loop trough frames
for frame in frames[1:-1]:
    frame_og = frame
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray_blur = cv2.GaussianBlur(frame_gray, (3, 3), 0)
    h, w = frame_gray.shape[:2]

    # # Sobel
    # # gradients
    # grad_x = cv2.Sobel(frame_gray_blur, cv2.CV_64F, 1, 0, ksize=3)
    # grad_y = cv2.Sobel(frame_gray_blur, cv2.CV_64F, 0, 1, ksize=3)
    # grad_x = cv2.convertScaleAbs(grad_x)
    # grad_y = cv2.convertScaleAbs(grad_y)
    # # weighted gradients
    # grad = cv2.addWeighted(grad_x, 0.5, grad_y, 0.5, 0)
    # # threshold
    # _, edges = cv2.threshold(grad, 0.2*255, 255, 0)

    # Edges: Canny
    edges = cv2.Canny(frame, 100, 200)

    # Bottom fill
    frame_filled = np.zeros((h, w), np.uint8)
    y_idxs = np.indices((h, w))[0]
    y_idxs_edges = y_idxs.copy()
    y_idxs_edges[edges == 0] = 0
    y_idxs_edges_max = np.argmax(y_idxs_edges, axis=0) # find the max y idx over each col
    idxs_after_edges = y_idxs >= y_idxs_edges_max
    frame_filled[idxs_after_edges] = 255
    # slower method:
    # for x in range(w):
    #     for y in range(h-1, -1, -1): # reverse order because y is 0 at the top
    #         if edges[y][x] < 255: frame_filled[y][x] = 255
    #         else: break

    # Erode
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (int(w/30), 1)) # horizontal erode kernal
    frame_filled_eroded = cv2.erode(frame_filled, erode_kernel)

    # Smoothen
    frame_filled_smooth = cv2.medianBlur(frame_filled_eroded, int(w/30))

    # Find path candidates
    y_idxs_area = y_idxs.copy()
    y_idxs_area[frame_filled_smooth == 0] = h+1
    y_idxs_area_max = np.argmin(y_idxs_area, axis=0)

    y_idxs_safest = np.min(y_idxs_area_max)
    x_idxs_safest = np.where(y_idxs_area_max == y_idxs_safest)[0]

    # Find best path candidate
    candidate_regions = []
    candidate_region = []
    candidates = []
    best_candidate = None
    best_candidate_size = 0
    prev_candidate = x_idxs_safest[0]
    for i, x in enumerate(x_idxs_safest[1:-1]):
        if x == prev_candidate + 1:
            candidate_region.append(x)
        if (x > prev_candidate + 1 or i == (x_idxs_safest[1:-1].size - 1)):
            if (len(candidate_region) > 0):
                candidate_regions.append(candidate_region)
                candidate = candidate_region[int(len(candidate_region)/2) - 1]
                candidates.append(candidate)
                candidate_size = len(candidate_region)
                if candidate_size > best_candidate_size:
                    best_candidate = candidate
                    best_candidate_size = candidate_size
                candidate_region = []
        prev_candidate = x

    if best_candidate == None:
        best_candidate = prev_candidate

    # print(len(candidate_regions))

    # Show path candidates
    frame_filled_path = cv2.cvtColor(frame_filled_smooth, cv2.COLOR_GRAY2BGR)
    for x in x_idxs_safest:
        cv2.circle(frame_filled_path, (x, y_idxs_safest), radius=3, color=(0, 0, 255), thickness=-1)
    for x in candidates:
        cv2.circle(frame_filled_path, (x, y_idxs_safest), radius=5, color=(255, 0, 0), thickness=-1)
    cv2.circle(frame_filled_path, (best_candidate, y_idxs_safest), radius=5, color=(0, 255, 0), thickness=-1)

    # Show best path
    cv2.arrowedLine(frame_og, (int(w/2), int(h/1)), (best_candidate, y_idxs_safest), (0, 255, 0), 2, tipLength=0.05)

    # Show
    imshow_array = [
        frame_og,
        np.full((h, 1, 3), [0, 0, 255], np.uint8),
        cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR),
        np.full((h, 1, 3), [0, 0, 255], np.uint8),
        frame_filled_path
    ]
    cv2.imshow('frame', np.hstack(imshow_array))
    k = cv2.waitKey(0) & 0xff
    if k == 27:
        break

cv2.destroyAllWindows()



