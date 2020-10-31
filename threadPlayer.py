
#!/usr/bin/env python3

import cv2
import base64
import os
import numpy as np
import queue
import time
import threading


# globals
outputDir    = 'frames'
clipFileName = 'clip.mp4'

def extract(extractedFrames, proConFull1,proConEmpty1,queueFirst):
  # initialize frame count
  count = 0

  # open the video clip
  vidcap = cv2.VideoCapture(clipFileName)

  # create the output directory if it doesn't exist
  '''if not os.path.exists(outputDir):
    print("Output directory {} didn't exist, creating".format(outputDir))
    os.makedirs(outputDir)'''

  # read one frame
  success,image = vidcap.read()
  print("Reading frame {} {} ".format(count, success))
  while success:
    # write the current frame out as a jpeg image
    success, jpgImage = cv2.imencode('.jpg', image)
    jpgAsText = base64.b64encode(jpgImage)
    #add frame to buffer 1
    proConEmpty1.acquire()
    queueFirst.acquire()
    extractedFrames.append(jpgAsText)
    queueFirst.release()
    proConFull1.release()
    success,image = vidcap.read()
    print('Reading frame {}'.format(count))
    count += 1

######################################################

def convert(extractedFrames, convertedFrames,proConFull1,proConEmpty1,queueFirst,proConFull2,
    proConEmpty2,queueSecond):
    # initialize frame count
    count = 0

    # get the next frame file name
    proConFull1.acquire()
    queueFirst.acquire()
    frameAsText = extractedFrames.pop(0)
    queueFirst.release()
    proConEmpty1.release()
    # decode the frame 
    jpgRawImage = base64.b64decode(frameAsText)
    jpgImage = np.asarray(bytearray(jpgRawImage), dtype=np.uint8)
    img = cv2.imdecode( jpgImage ,cv2.IMREAD_UNCHANGED)


    while img is not None:
        print("Converting frame {}".format(count))

        # convert the image to grayscale
        grayscaleFrame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #img to text
        success, jpgImage = cv2.imencode('.jpg', grayscaleFrame)
        jpgAsText = base64.b64encode(jpgImage) 

        # add img to buffer 2
        proConEmpty2.acquire()
        queueFirst.acquire()
        convertedFrames.append(jpgAsText)
        queueFirst.release()
        proConFull2.release()

        count += 1

        # get the next frame from buffer 1
        proConFull1.acquire()
        queueFirst.acquire()
        frameAsText = extractedFrames.pop(0)
        queueFirst.release()
        proConEmpty1.release()
        # decode the frame 
        jpgRawImage = base64.b64decode(frameAsText)
        jpgImage = np.asarray(bytearray(jpgRawImage), dtype=np.uint8)
        img = cv2.imdecode( jpgImage ,cv2.IMREAD_UNCHANGED)
#####################################################################

def display(convertedFrames,proConFull2,proConEmpty2,queueFirst):
    frameDelay   = 42       # the answer to everything

    # initialize frame count
    count = 0

    startTime = time.time()
    #grab first frame
    proConFull2.acquire()
    queueFirst.acquire()
    frameAsText = convertedFrames.pop(0)
    queueFirst.release()
    proConEmpty2.release()
     # decode the frame 
    jpgRawImage = base64.b64decode(frameAsText)
    jpgImage = np.asarray(bytearray(jpgRawImage), dtype=np.uint8)
    img = cv2.imdecode( jpgImage ,cv2.IMREAD_UNCHANGED)

    while img is not None:
        
        print("Displaying frame {}".format(count))
        # Display the frame in a window called "Video"
        cv2.imshow("Video", img)

        # compute the amount of time that has elapsed
        # while the frame was processed
        elapsedTime = int((time.time() - startTime) * 1000)
        print("Time to process frame {} ms".format(elapsedTime))
        
        # determine the amount of time to wait, also
        # make sure we don't go into negative time
        timeToWait = max(1, frameDelay - elapsedTime)

        # Wait for 42 ms and check if the user wants to quit
        if cv2.waitKey(timeToWait) and 0xFF == ord("q"):
            break    

        # get the start time for processing the next frame
        startTime = time.time()
        
        # get next frame from buffer 2
        count += 1
        proConFull2.acquire()
        queueFirst.acquire()
        frameAsText = convertedFrames.pop(0)
        queueFirst.release()
        proConEmpty2.release()
        # decode the frame 
        jpgRawImage = base64.b64decode(frameAsText)
        jpgImage = np.asarray(bytearray(jpgRawImage), dtype=np.uint8)
        img = cv2.imdecode( jpgImage ,cv2.IMREAD_UNCHANGED)

    # make sure we cleanup the windows, otherwise we might end up with a mess
    cv2.destroyAllWindows()

##########################################

#start threads 

#First producer-consumer buffer
#array to store frames extracted from video
#size of the buffers limit both empty and full
extractedFrames = []
queueFirst = threading.Lock()
proConEmpty1 = threading.Semaphore(10)
proConFull1 = threading.Semaphore(0)

#second producer-consumer buffer
#array to store grayscale frames
convertedFrames = []
queueSecond = threading.Lock()
proConEmpty2 = threading.Semaphore(10)
proConFull2 = threading.Semaphore(0)


#need to manage the excecution of 3 threads 
#instantiate the lock for each method using target and then call .start() to begin the function for the thread

#thread one targets the method to extract the frames pass the parameters of the extracted frames array, the semaphore for the 
#full and empty "baskets" buffer and the lock object for thread 1
#the convert to greyscale can use either thread1 or 2 which ever is not locked
thread1 = threading.Thread(target = extract, args = (extractedFrames, proConFull1,proConEmpty1,queueFirst))
thread2 = threading.Thread(target = convert, args = (extractedFrames, convertedFrames,proConFull1,proConEmpty1,queueFirst,proConFull2,
    proConEmpty2,queueSecond))
thread3 = threading.Thread(target = display, args = (convertedFrames,proConFull2,proConEmpty2,queueSecond))


thread1.start()
thread2.start()
thread3.start()



