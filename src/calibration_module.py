#! /usr/bin/env python
import numpy as np
import cv2
import glob
import yaml


class CameraCalib :
    def __init__(self,img_path='/tmp',CHESSX=8,CHESSY=6,extension=".jpg"):
        """
        Initialize Camera Calibration Class
        @param: img_path = [path to get images], CHESSX = [chessboard corners in X direction ]
                CHESSY = [chessboard corners in Y direction] 
        """
        self.img_path = img_path
        self.chessx = CHESSX
        self.chessy = CHESSY
        self.data = {}
        self.file_extension = extension
    
    def show_image(self,image,time=1000):
        """
        Image Visualization for [time] msecs.
        @param: image, time [in msecs]
        """
        y = 540
        x = 1.5*y
        imS = cv2.resize(image, (int(x), y))  # Resize image
        cv2.imshow("output", imS)
        cv2.waitKey(time)
        
    def calcReprojectionError(self, objpoints, imgpoints, mtx, dist, rvecs, tvecs):
        mean_error = 0
        for i in xrange(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("Re-projection Error: {}".format(mean_error / len(objpoints)))
    
    def compute(self,visualization=True,save_yaml=True):
        """
        Camera calibration and camera matrix computation.
        @param: visualization = [True|False] to enable imgs visualization, 
                save_yaml = [True|False] to save image in a yaml file.
        """
        
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.chessy*self.chessx,3), np.float32)
        objp[:,:2] = np.mgrid[0:self.chessx,0:self.chessy].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob(self.img_path+'/*'+self.file_extension)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            
            # show grey image 
            if(visualization):
                self.show_image(gray)
            
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.chessx,self.chessy),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (self.chessx,self.chessy), corners2,ret)
                if(visualization):
                    self.show_image(img)

        cv2.destroyAllWindows()

        # calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # transform the matrix and distortion coefficients to writable lists
        self.data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
        
        self.calcReprojectionError(objpoints,imgpoints,mtx,dist,rvecs,tvecs)
        
        # print results
        print("Camera Calibration Matrix:\n",self.data)

        # and save it to a file
        if (save_yaml):
            with open("calibration_matrix.yaml", "w") as f:
                yaml.dump(self.data, f)

if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("-cw", "--chessboard_width", required=False, default="8", help="number of intersections in x axis")
    ap.add_argument("-ch", "--chessboard_height", required=False, default="6", help="number of intersections in y axis")
    ap.add_argument("-sd", "--square_dimension", required=False, default="0.026", help="square dimension in meters")
    ap.add_argument("-p", "--path", required=True, help="path to images folder")
    ap.add_argument("-e", "--file_extension", required=False, default=".jpg", help="extension of images")
    ap.add_argument("-a", "--auto_mode", required=False, default="True", \
                    help="automatic mode uses all images inside images folder to run calibration")
    args = vars(ap.parse_args())

    auto_mode = eval(args["auto_mode"])
    CHESSBOARD_WIDTH = int(args["chessboard_width"])
    CHESSBOARD_HEIGHT = int(args["chessboard_height"])
    CALIBRATION_SQUARE_DIMENSION = float(args["square_dimension"]) # meters
    
    # initialize class
    cam_calibration = CameraCalib(img_path=args["path"],CHESSX=CHESSBOARD_WIDTH, CHESSY=CHESSBOARD_HEIGHT,extension=args["file_extension"])

    # Compute Calibration
    cam_calibration.compute(True)
