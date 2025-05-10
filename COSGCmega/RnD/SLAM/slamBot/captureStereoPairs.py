import cv2
import os

camL = cv2.VideoCapture(0)  # Adjust if USB cam is 1
camR = cv2.VideoCapture(1)  # Swap if needed

pair_count = 0
os.makedirs("calib/left", exist_ok=True)
os.makedirs("calib/right", exist_ok=True)

print("Press SPACE to capture a pair. ESC to exit.")

while True:
    retL, imgL = camL.read()
    retR, imgR = camR.read()

    both = cv2.hconcat([imgL, imgR])
    cv2.imshow("Stereo View", both)

    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break
    elif key == 32:  # SPACE
        cv2.imwrite(f"calib/left/left_{pair_count:02}.png", imgL)
        cv2.imwrite(f"calib/right/right_{pair_count:02}.png", imgR)
        print(f"Captured pair {pair_count}")
        pair_count += 1

camL.release()
camR.release()
cv2.destroyAllWindows()
