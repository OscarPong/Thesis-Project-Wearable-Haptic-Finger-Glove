import threading
import socket
import cv2
from cvzone.HandTrackingModule import HandDetector



# -------------- CONFIG --------------

# For BLE
BLE_ADDRESS = "28:37:2f:76:e9:36"
CHAR_UUID = "87654321-4321-4321-4321-abcdefabcd01"      # BLE characteristic UUID

# For UDP
SEND_PORT = 5021  # Camera → Unity

# Camera + Unity UDP setup
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverAddressPort = ("127.0.0.1", SEND_PORT)


# -------------- CAMERA THREAD FUNCTION --------------

def hand_tracking_loop():
    width, height = 1280, 720
    cap = cv2.VideoCapture(0)
    cap.set(3, width)
    cap.set(4, height)
    detector = HandDetector(maxHands=1, detectionCon=0.8)

    while True:
        success, img = cap.read()
        hands, img = detector.findHands(img)
        data = []
        # Landmark values - (x,y,z) * 21
        if hands:
            hand = hands[0]
            lmList = hand['lmList']
            for lm in lmList:
                data.extend([lm[0], height - lm[1], lm[2]])     # Modify the y-value and convert to one list

            f = 650
            W = 8.4
            w = abs(data[51] - data[15])
            if w != 0:
                d = (W * f) / w
                for i in range(0, 21):
                    # Add z offset and scale up to account for unity scale
                    data[i*3 + 2] += int(d * 10)
                    # Compensate x,y scaling due to z offset
                    data[i*3] *= d/100
                    data[i*3 + 1] *= d/100

            sock_send.sendto(str.encode(str(data)), serverAddressPort)

        img = cv2.resize(img, (0, 0), None, 0.5, 0.5)
        cv2.imshow("Image", img)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    
if __name__ == "__main__":
    # Start camera/UDP sending thread
    cam_thread = threading.Thread(target=hand_tracking_loop)
    cam_thread.start()
