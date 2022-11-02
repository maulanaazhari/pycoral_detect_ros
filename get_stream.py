import cv2
import imagezmq
import datetime
import numpy as np
import sys

image_hub = imagezmq.ImageHub(open_port='tcp://127.0.0.1:5555', REQ_REP=False)
filename = f'videos/from_stream/{datetime.datetime.now().strftime("%Y%m%d-%H%M%S")}.avi'
first_image = True

while True:  # show streamed images until Ctrl-C
    # disp_name, image = image_hub.recv_image()

    try:
        sent_from, jpg_buffer = image_hub.recv_jpg()
        image = cv2.imdecode(np.frombuffer(jpg_buffer, dtype=np.uint8), -1)
        if first_image:
            fourcc = cv2.VideoWriter_fourcc(*'DIVX')
            out = cv2.VideoWriter(filename, fourcc, 25.0, (image.shape[1], image.shape[0]))
            first_image = False

        out.write(image)
        cv2.imshow(sent_from, image) # 1 window for each RPi
        cv2.waitKey(1)

    except KeyboardInterrupt:
        out.release()
        cv2.destroyAllWindows()
        print(f"Video {filename} is saved!!!!!")
        sys.exit()