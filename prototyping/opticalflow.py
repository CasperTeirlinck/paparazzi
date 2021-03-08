import cv2
import os
import glob
import numpy as np

# data directory name
FRAMESDIR = 'cyberzoo_poles/20190121-135009'
PATH = os.path.dirname(os.path.realpath(__file__))

# load and sort frames from data directory
frames = glob.glob(PATH+'/AE4317_2019_datasets/'+FRAMESDIR+'/*.jpg')
frames.sort()
frames = [cv2.rotate(cv2.imread(img), cv2.ROTATE_90_COUNTERCLOCKWISE)  for img in frames]

# Lucas Kanade setup
lk_params = {
    'winSize': (15, 15),
    'maxLevel': 2,
    'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
}

# Corner detection setup
corner_params = {
    'maxCorners': 20,
    'qualityLevel': 0.3,
    'minDistance': 7,
    'blockSize': 7
}

color = np.random.randint(0, 255, (100, 3))

# init loop
frame_prev = frames[0]
p0 = cv2.goodFeaturesToTrack(cv2.cvtColor(frame_prev, cv2.COLOR_BGR2GRAY), mask=None, **corner_params)
mask = np.zeros_like(frame_prev)

# loop trough frames
for frame in frames[1:-1]:
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray_prev = cv2.cvtColor(frame_prev, cv2.COLOR_BGR2GRAY)

    # calc optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(frame_gray_prev, frame_gray, p0, None, **lk_params)

    # select good points
    p1_good = p1[st==1]
    p0_good = p0[st==1]

    # update prev
    frame_gray_prev = frame_gray.copy()
    p0 = p1_good.reshape(-1, 1, 2)

    # draw optical flow
    for i, (new, old) in enumerate(zip(p1_good, p0_good)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
        frame = cv2.circle(frame,(a,b),5, color[i].tolist(), -1)
    img = cv2.add(frame, mask)

    # show img
    cv2.imshow('frame', img)
    k = cv2.waitKey(0) & 0xff
    if k == 27:
        break

cv2.destroyAllWindows()



