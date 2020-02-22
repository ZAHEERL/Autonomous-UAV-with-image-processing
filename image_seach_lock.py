import cv2
import numpy as np
import math
import time

target1 = cv2.imread("C:/Users/asus/Desktop/drone/STM_logo.png", cv2.IMREAD_GRAYSCALE)
target2 = cv2.imread("C:/Users/asus/Desktop/drone/METU_logo_jpg.jpg", cv2.IMREAD_GRAYSCALE)
target3 = cv2.imread("C:/Users/asus/Desktop/drone/ORT_logo.png", cv2.IMREAD_GRAYSCALE)
target4 = cv2.imread("C:/Users/asus/Desktop/drone/LAND_logo.png", cv2.IMREAD_GRAYSCALE)
home = cv2.imread("C:/Users/asus/Desktop/drone/TR_flag.png", cv2.IMREAD_GRAYSCALE)

def search_lock(target, algo, precision):
    #Capturing Real Time Video
    cap = cv2.VideoCapture(0)
    #width
    cap.set(3,432)
    #height
    cap.set(4,432)
    if(algo == "orb"):
        #ORB
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(target, None)
        output = cv2.drawKeypoints(target, keypoints, None)

        #Flann parameters for ORB
        indexparameters= dict(algorithm = 6,
                     table_number = 12,#6, # 12
                     key_size = 20,#12,     # 20
                     multi_probe_level = 2)#1) #2

        searchparameters = dict(checks=30)
        flann = cv2.FlannBasedMatcher(indexparameters, searchparameters)

    #for  SURF
    if(algo == "surf"):
        #SURF
        surf = cv2.xfeatures2d.SURF_create() # FEATURES
        keypoints, descriptors = surf.detectAndCompute(target, None) # KEYPOINTS
        output = cv2.drawKeypoints(target, keypoints, None)   #outImage	=	cv.drawKeypoints(	image, keypoints, outImage[, color[, flags]]	)

        #Flann parameters for SURF
        indexparameters1 = dict(algorithm= 0 , trees = 5) # 
        searchparameters1 = dict()
        flann = cv2.FlannBasedMatcher(indexparameters1, searchparameters1)

    

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Create some random colors
    color = np.random.randint(0,255,(100,3))

    if cap.isOpened():
        ret , frame = cap.read()
        #cap.read() returns a bool (True/False). If frame is read correctly, it will be True. So you can check end of the video by checking this return value.
        #ret will store that bool value 

    else:
        ret = False

    while ret:
        cv2.imshow("ORB-TARGET", output)

        ret , frame = cap.read()
        #frame2 = frame
        #cv2.imshow("Live Video Feed",frame)
        #gaussianBlur = cv2.GaussianBlur(frame,(5,5),0)
        medianBlur = cv2.medianBlur(frame,5)
        grayFrame = cv2.cvtColor(medianBlur,cv2.COLOR_BGR2GRAY)   #GRAYSCALE

        if (algo == "orb"):
            print("ORB ALGORITHM ")
            #ORB ALGORITHM APPLIED TO REAL TIME CAPTURING
            keypoints_grayFrame, descriptors_grayFrame = orb.detectAndCompute(grayFrame, None)
            show_keypoints_grayFrame = cv2.drawKeypoints(grayFrame,keypoints_grayFrame, None)
            #cv2.imshow("Real Time Cap orb", show_keypoints_grayFrame_orb)

        if (algo == "surf"):
            print("SURF ALGORITHM ")
            #SURF ALGORITHM APPLIED TO REAL TIME CAPTURING
            keypoints_grayFrame, descriptors_grayFrame = surf.detectAndCompute(grayFrame, None)
            show_keypoints_grayFrame = cv2.drawKeypoints(grayFrame,keypoints_grayFrame, None)
            cv2.imshow("Real Time Cap surf", show_keypoints_grayFrame)

        matches_flann = flann.knnMatch(descriptors, descriptors_grayFrame, k=2)
        
        goodMatches_flann = []
        for m in matches_flann:
            if len(m) > 0 and m[0].distance < precision *m[-1].distance:
                goodMatches_flann.append(m[0])

        
        result_flann = cv2.drawMatches(target, keypoints, grayFrame, keypoints_grayFrame, goodMatches_flann, grayFrame)

        #cv2.imshow("Result_flann_orb", result_flann_orb)

        if len(goodMatches_flann) > 7:
            cv2.destroyWindow("Result")
            cv2.putText(result_flann,'TARGET-DETECTED',(650,100), cv2.FONT_HERSHEY_SIMPLEX, .5,(0,0,255),2,cv2.LINE_AA)
            cv2.imshow("Target-detected", result_flann)

            
            queryPoints = np.float32([keypoints[i.queryIdx].pt for i in goodMatches_flann]).reshape(-1, 1, 2)   # extracting location of good matches from targeted image
            trainPoints = np.float32([keypoints_grayFrame[j.trainIdx].pt for j in goodMatches_flann]).reshape(-1, 1, 2) # extracting location of good matches from real time vision
            #print(queryPoints)
            #representativematrix, maskk = cv2.findHomography(queryPoints, trainPoints, cv2.RANSAC, 5.0)  # this matrix represents location of target in real time vision
            #matchesMask = maskk.ravel().tolist()
            #height, width = target.shape  # height and width of original targeted image
            #points = np.float32([[0, 0],[0, height],[width, height],[width,0]]).reshape(-1, 1, 2)
            #adaptiveTemplate = cv2.perspectiveTransform(points, representativematrix)  # points will adapt matrix
            #homography = cv2.polylines(frame, [np.int32(adaptiveTemplate)], True, (255,0,0), 3)

            #cv2.imshow("Homograpyh", homography)
            
            targetPoints= trainPoints
            #cv2.goodFeaturesToTrack(image, maxCorners, qualityLevel, minDistance[, corners[, mask[, blockSize[, useHarrisDetector[, k]]]]]) → corners
            #STMpoints = cv2.goodFeaturesToTrack(grayFrame, mask=None, **feature_params)
            
            # Create a mask image for drawing purposes
            mask = np.zeros_like(frame)

            while(len(goodMatches_flann) > 7):
                ret,frame2 = cap.read()
                newGrayFrame = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

                # calculate optical flow
                flowMatches, st, err = cv2.calcOpticalFlowPyrLK(grayFrame, newGrayFrame, targetPoints, None, **lk_params)

                # Select good points
                try:
                    good_new = flowMatches[st==1]
                    good_old = targetPoints[st==1]
                except:
                    break

                # draw the tracks
                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    print("new points",(a,b))
                    c,d = old.ravel()
                    print("old points",(c,d))
                    maskk= cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
                    frame3 = cv2.circle(frame2,(a,b),5,color[i].tolist(),-1)

                    """
                    if (c-a)>0:
                        print("*******DOWN********")
                    elif (c-a)<0:
                        print("*******UP********")
                    elif (d-b) >0:
                        print("*******RIGHT*******")
                    elif (d-b) <0:
                        print("*******LEFT*******")
                    """ 
                
                    if not((150<a and a<195) and (115<b and b<155)):  #160 middle     145middle #(130<a and a<220) and (114<b and b<190)
                        if(a < 130):
                            print("RIGHT!")
                        elif(a > 220):
                            print("LEFT!")
                        elif(b < 114):
                            print("BACK!")
                        elif(b > 190):
                            print("FRONT!")
                    else:
                        cv2.putText(frame3,'Initializing Landing...',(80,80), cv2.FONT_HERSHEY_SIMPLEX, .5,(30,40,50),2,cv2.LINE_AA)
                        
                        print("Initiate Landing...")
                    
                """    
                print(maskk.shape)
                print(frame3.shape)    
                print(maskk.dtype)
                print(frame3.dtype)
                """
                img = cv2.add(frame3,maskk)
                img= cv2.rectangle(img, (100,95), (250,175),(255,0, 0), 2)
                img= cv2.rectangle(img, (120,115), (230,155),(0,255, 0), 2)    #(0,10), (350,275)
                cv2.imshow('frame',img)
                # Now update the previous frame and previous points
                grayFrame = newGrayFrame.copy()
                targetPoints = good_new.reshape(-1,1,2)
                if cv2.waitKey(1) == 27:
                    break
        else:
            cv2.destroyWindow("Target-detected")
            cv2.destroyWindow("frame")
            #cv2.destroyWindow("Homograpyh")
            cv2.imshow("Result", result_flann)

        if cv2.waitKey(1) == 27:
                break

    # When everything done, release the capture
    cv2.destroyAllWindows()
    cap.release()
search_lock(target1, "orb", 0.5)

#search_lock(target3, "orb", 0.68)
#search_lock(target4, "orb", 0.6)
#search_lock(home, "orb", 0.7)


