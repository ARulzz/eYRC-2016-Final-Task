import cv2
cv2.NamedWindow("w1", cv.CV_WINDOW_AUTOSIZE)
capture = cv2.CaptureFromCAM(1)

def repeat():
    frame = cv.QueryFrame(capture)
    cv.ShowImage("w1", frame)

while True:
    repeat()
    if cv.WaitKey(33)==27:
        break

cv.DestroyAllWindows()
