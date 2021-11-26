import cv2
import numpy as np
import sys
import os

def get_dim(bgr_image):
    d = bgr_image.shape
    h = d[0]
    w = d[1]
    return d, h, w

def open_videoCapture(filename):
    video = cv2.VideoCapture(filename)
    if not video.isOpened():
        print("Cannot open video")
        sys.exit(1)
    return video

def read_frame(videoCapture, resize_factor):
    ok, image = videoCapture.read()
    if not ok:
        print("Cannot read video file")
        sys.exit(1)
    _, h, w = get_dim(image)
    image = cv2.resize(image, (int(w * resize_factor), int(h * resize_factor)))
    return image

def init_tracker(tracker, firstFrame):
    bbox = cv2.selectROI(firstFrame, False)
    ok = tracker.init(firstFrame, bbox)
    if not ok:
        print("Cannot initialize tracker")
        sys.exit(1)
    cv2.destroyAllWindows()
    return tracker

def main():
    # main()

    tracker = cv2.TrackerKCF_create()
    # Read video
    video = cv2.VideoCapture("WIN_20210827_11_04_42_Pro.mp4")

    # Exit if video not opened.
    if not video.isOpened():
        print
        "Could not open video"
        sys.exit()

    # Read first frame.
    frame = read_frame(video, 0.5)

    # Define an initial bounding box
    bbox = (287, 23, 86, 320)

    for i in range(120):
        frame = read_frame(video, 0.5)

    # Uncomment the line below to select a different bounding box
    bbox = cv2.selectROI(frame, False)

    # Initialize tracker with first frame and bounding box
    ok = tracker.init(frame, bbox)

    # video = cv2.VideoCapture("WIN_20210827_11_02_24_Pro.mp4")

    # create the output video
    dim, height, width = get_dim(frame)
    fourcc = cv2.VideoWriter_fourcc('W', 'M', 'V', '2')
    videoWriter = cv2.VideoWriter("tracker_output.wmv", fourcc=fourcc, fps=24.0, frameSize=(width, height))

    while True:
        # Read a new frame
        frame = read_frame(video, 0.5)

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        # Display tracker type on frame
        # cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display result
        cv2.imshow("Tracking", frame)

        videoWriter.write(frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break

    videoWriter.close()

if __name__ == '__main__':
    main()

