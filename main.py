__author__ = 'ravil'
from PySide.QtCore import *
from PySide.QtGui import *
import cv2 as cv
import glob
from invkin import *
from servodrv import *

class RoboPickApp(QWidget):

    CAM_1 = 1
    CAM_2 = 2

    OBJECT_H = 20.0

    CHESS_SQUARE_SIZE = 25
    POINTS_ALR = 9
    POINTS_ALC = 6
    ROBOT_ARM_X = 320
    ROBOT_ARM_Y = 50
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    PLATE_X = 475
    PLATE_Y = 50
    SYS2_X = 654.0  # 652
    SYS2_Y = 376.0  # 374

    PURPLE = (176, 39, 156)
    RED = (54, 67, 244)
    GREEN = (80, 175, 76)
    BLUE = (243, 150, 33)
    ORANGE = (34, 87, 255)

    def __init__(self):
        self.qt_app = QApplication(sys.argv)
        QWidget.__init__(self, None)
        self.scr_height = self.qt_app.desktop().screenGeometry().height()
        self.scr_width = self.qt_app.desktop().screenGeometry().width()
        print ""

        self.com_edit = QTextEdit()
        self.com_edit.append('First forwarding will cause servos to default positions\n')
        self.servodrv = ServoDrv(self.com_edit)

        self.setWindowTitle('RoboPickApp')
        self.setMinimumSize(400, 0)
        self.move(100, 100)
        self.vbox1 = QVBoxLayout()
        self.vbox2 = QVBoxLayout()
        self.vbox3 = QVBoxLayout()
        self.vbox4 = QVBoxLayout()
        self.hbox = QHBoxLayout()

        self.camera_combo = QComboBox(self)
        self.camera_combo.addItem("Camera 1", self.CAM_1)
        self.camera_combo.addItem("Camera 2", self.CAM_2)
        self.camera_combo.setCurrentIndex(0)

        self.detect_button = QPushButton('&Detect object')
        self.detect_button.clicked.connect(self.detect)

        self.capture_button = QPushButton('&Capture frames')
        self.capture_button.clicked.connect(self.capture)

        self.intrinsics_button = QPushButton('&Intrinsic matrices')
        self.intrinsics_button.clicked.connect(self.intrinsics)

        self.rtr_button = QPushButton('&Rotation/translation')
        self.rtr_button.clicked.connect(self.rtrcam)

        self.augreal_button = QPushButton('&Augmented reality')
        self.augreal_button.clicked.connect(self.augreal)

        self.opencom_button = QPushButton('&Open ' + self.servodrv.COM_PORT + ' port')
        self.opencom_button.clicked.connect(self.open_com)

        self.closecom_button = QPushButton('&Exit')
        self.closecom_button.clicked.connect(self.close_com)

        self.send_button = QPushButton('&Send package')
        self.send_button.clicked.connect(self.send_package)

        self.def_button = QPushButton('&Go to default')
        self.def_button.clicked.connect(self.default)

        self.idle_button = QPushButton('&Nothing to do')
        self.idle_button.clicked.connect(self.idle)

        self.srv1_edit = QLineEdit('254')
        self.srv1_edit.setValidator(QIntValidator(0, 255, self))
        self.srv1_label = QLabel("Base")
        self.srv1_box = QHBoxLayout()
        self.srv1_box.addWidget(self.srv1_label)
        self.srv1_box.addWidget(self.srv1_edit)

        self.srv2_edit = QLineEdit('254')
        self.srv2_edit.setValidator(QIntValidator(0, 255, self))
        self.srv2_label = QLabel("Legs")
        self.srv2_box = QHBoxLayout()
        self.srv2_box.addWidget(self.srv2_label)
        self.srv2_box.addWidget(self.srv2_edit)

        self.srv3_edit = QLineEdit('254')
        self.srv3_edit.setValidator(QIntValidator(0, 255, self))
        self.srv3_label = QLabel("Waist")
        self.srv3_box = QHBoxLayout()
        self.srv3_box.addWidget(self.srv3_label)
        self.srv3_box.addWidget(self.srv3_edit)

        self.srv4_edit = QLineEdit('254')
        self.srv4_edit.setValidator(QIntValidator(0, 255, self))
        self.srv4_label = QLabel("Neck")
        self.srv4_box = QHBoxLayout()
        self.srv4_box.addWidget(self.srv4_label)
        self.srv4_box.addWidget(self.srv4_edit)

        self.srv5_edit = QLineEdit('254')
        self.srv5_edit.setValidator(QIntValidator(0, 255, self))
        self.srv5_label = QLabel("Wrist")
        self.srv5_box = QHBoxLayout()
        self.srv5_box.addWidget(self.srv5_label)
        self.srv5_box.addWidget(self.srv5_edit)

        self.srv6_edit = QLineEdit('254')
        self.srv6_edit.setValidator(QIntValidator(0, 255, self))
        self.srv6_label = QLabel("Grip")
        self.srv6_box = QHBoxLayout()
        self.srv6_box.addWidget(self.srv6_label)
        self.srv6_box.addWidget(self.srv6_edit)

        self.x_edit = QLineEdit('')
        self.x_edit.setValidator(QDoubleValidator(0, 720, 2, self))
        self.x_label = QLabel("x")
        self.x_box = QHBoxLayout()
        self.x_box.addWidget(self.x_label)
        self.x_box.addWidget(self.x_edit)

        self.y_edit = QLineEdit('')
        self.y_edit.setValidator(QDoubleValidator(0, 720, 2, self))
        self.y_label = QLabel("y")
        self.y_box = QHBoxLayout()
        self.y_box.addWidget(self.y_label)
        self.y_box.addWidget(self.y_edit)

        self.move_button = QPushButton("&Move servos")
        self.move_button.clicked.connect(self.move_servo)

        self.vbox2.setAlignment(Qt.AlignTop)
        self.vbox2.addWidget(self.detect_button)
        self.vbox2.addSpacing(20)
        self.vbox2.addWidget(self.capture_button)
        self.vbox2.addWidget(self.intrinsics_button)
        self.vbox2.addWidget(self.rtr_button)
        self.vbox2.addWidget(self.augreal_button)
        self.vbox2.addSpacing(20)
        self.vbox2.addWidget(self.opencom_button)
        self.vbox2.addWidget(self.closecom_button)

        self.vbox3.setAlignment(Qt.AlignTop)
        self.vbox3.setContentsMargins(10, 0, 0, 0)
        self.vbox3.addLayout(self.srv1_box)
        self.vbox3.addLayout(self.srv2_box)
        self.vbox3.addLayout(self.srv3_box)
        self.vbox3.addLayout(self.srv4_box)
        self.vbox3.addLayout(self.srv5_box)
        self.vbox3.addLayout(self.srv6_box)
        self.vbox3.addSpacing(10)
        self.vbox3.addWidget(self.send_button)
        self.vbox3.addWidget(self.def_button)
        self.vbox3.addWidget(self.idle_button)

        self.vbox4.setAlignment(Qt.AlignTop)
        self.vbox4.addWidget(self.camera_combo)
        self.vbox4.addSpacing(10)
        self.vbox4.addLayout(self.x_box)
        self.vbox4.addLayout(self.y_box)
        self.vbox4.addWidget(self.move_button)

        self.hbox.addLayout(self.vbox2)
        self.hbox.addLayout(self.vbox3)
        self.hbox.addLayout(self.vbox4)

        self.vbox1.addLayout(self.hbox)
        self.vbox1.addSpacing(10)
        self.vbox1.addWidget(self.com_edit)

        self.setLayout(self.vbox1)

    def nothing(self, x):
        pass

    def detect(self):
        print('%s %s' % ('Colour', 'based object detection'))
        font_face = cv.QT_FONT_NORMAL
        font_scale = 1
        thickness = 2
        index = 1

        cap1 = cv.VideoCapture(self.CAM_1)
        cap1.set(3, self.FRAME_WIDTH)
        cap1.set(4, self.FRAME_HEIGHT)

        cap2 = cv.VideoCapture(self.CAM_2)
        cap2.set(3, self.FRAME_WIDTH)
        cap2.set(4, self.FRAME_HEIGHT)

        cv.namedWindow('Mask1', cv.WINDOW_AUTOSIZE)
        cv.namedWindow('Mask2', cv.WINDOW_AUTOSIZE)
        cv.createTrackbar('Lower hue', 'Mask1', 0, 179, self.nothing)
        cv.createTrackbar('Lower saturation', 'Mask1', 0, 255, self.nothing)
        cv.createTrackbar('Lower value', 'Mask1', 0, 255, self.nothing)
        cv.setTrackbarPos('Lower hue', 'Mask1', 35)
        cv.setTrackbarPos('Lower saturation', 'Mask1', 90)
        cv.setTrackbarPos('Lower value', 'Mask1', 85)

        cv.createTrackbar('Upper hue', 'Mask1', 0, 179, self.nothing)
        cv.createTrackbar('Upper saturation', 'Mask1', 0, 255, self.nothing)
        cv.createTrackbar('Upper value', 'Mask1', 0, 255, self.nothing)
        cv.setTrackbarPos('Upper hue', 'Mask1', 90)
        cv.setTrackbarPos('Upper saturation', 'Mask1', 180)
        cv.setTrackbarPos('Upper value', 'Mask1', 180)

        cv.namedWindow('Webcam', cv.WINDOW_AUTOSIZE)
        cv.moveWindow('Mask1', self.scr_width - self.FRAME_WIDTH, 0)
        cv.moveWindow('Mask2', self.scr_width - self.FRAME_WIDTH, self.scr_height - self.FRAME_HEIGHT)
        cv.moveWindow('Webcam', self.scr_width - self.FRAME_WIDTH - 100, 100)

        while cap1.isOpened() and cap2.isOpened():
            (ret1, frame1) = cap1.read()
            (ret2, frame2) = cap2.read()
            if ret1 and ret2:
                hsv1 = cv.cvtColor(frame1, cv.COLOR_BGR2HSV)
                hsv2 = cv.cvtColor(frame2, cv.COLOR_BGR2HSV)

                lh = cv.getTrackbarPos('Lower hue', 'Mask1')
                ls = cv.getTrackbarPos('Lower saturation', 'Mask1')
                lv = cv.getTrackbarPos('Lower value', 'Mask1')
                uh = cv.getTrackbarPos('Upper hue', 'Mask1')
                us = cv.getTrackbarPos('Upper saturation', 'Mask1')
                uv = cv.getTrackbarPos('Upper value', 'Mask1')

                lower_boundary = np.array([lh, ls, lv])
                upper_boundary = np.array([uh, us, uv])
                mask1 = cv.inRange(hsv1, lower_boundary, upper_boundary)
                mask2 = cv.inRange(hsv2, lower_boundary, upper_boundary)

                cv.imshow('Mask1', mask1)
                cv.imshow('Mask2', mask2)

                # res = cv.bitwise_and(frame, frame, mask=mask)
                # gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
                # ret, thresh = cv.threshold(gray, 127, 255, 0)

                _, contours1, _ = cv.findContours(mask1, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                _, contours2, _ = cv.findContours(mask2, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
                # cnt = contours[0]
                # frame1 = cv.drawContours(frame1, contours1, -1, (0, 255, 0), 3)

                if not contours1:
                    print "Contours on the camera 1 not found!"
                if not contours2:
                    print "Contours on the camera 2 not found!"

                rects = {}

                i = 0
                while i < len(contours1):
                    x, y, w, h = cv.boundingRect(contours1[i])
                    rects[i] = [x, y, w, h, self.CAM_1]
                    i += 1

                while i < len(contours1) + len(contours2):
                    x, y, w, h = cv.boundingRect(contours2[i - len(contours1)])
                    rects[i] = [x, y, w, h, self.CAM_2]
                    i += 1

                if len(rects) is not 0:
                    max_rect = max(rects.iterkeys(), key=(lambda key: rects[key][2] * rects[key][3]))
                    x = rects[max_rect][0]
                    y = rects[max_rect][1]
                    w = rects[max_rect][2]
                    h = rects[max_rect][3]
                    index = rects[max_rect][4]

                    if index == self.CAM_1:
                        frame = frame1
                    elif index == self.CAM_2:
                        frame = frame2

                    frame = cv.rectangle(frame, (x, y), (x+w, y+h), self.ORANGE, 2)
                    u = x + w/2
                    v = y + h/2
                    frame = cv.circle(frame, (u, v), 2, self.ORANGE, 2)
                else:
                    frame = np.zeros((self.FRAME_HEIGHT, self.FRAME_WIDTH, 3), np.uint8)

                k = cv.waitKey(1)
                if k & 0xFF == ord('q'):
                    break
                elif k & 0xFF == ord('c') and len(rects) is not 0:
                    frame = self.draw_axes(frame, index=index, output=False, from_file=True, length=5)

                    world = self.proj_to_world((u, v), index=index)
                    X = world[0]
                    Y = world[1]

                    height, _ = frame.shape[:2]
                    height -= 15
                    frame = cv.putText(frame, 'X: ' + str(X) + '; Y: ' + str(Y), (10, height),
                                       font_face, font_scale, self.PURPLE, thickness)
                    if index is not self.CAM_2:
                        frame, imgpts = self.draw_lines(frame, (X, Y), (u, v), index=index, thickness=2)
                        frame = cv.putText(frame, str(round(X / 10, 2)),
                                           (int(imgpts[0].ravel()[0]), int(imgpts[0].ravel()[1])),
                                           font_face, font_scale, self.PURPLE, thickness)
                        frame = cv.putText(frame, str(round(Y / 10, 2)),
                                           (int(imgpts[1].ravel()[0]), int(imgpts[1].ravel()[1])),
                                           font_face, font_scale, self.PURPLE, thickness)
                    cv.imshow('Webcam', frame)
                    cv.imwrite('webcam.png', frame)

                    comp_thread = ComputingThread((X, Y), (self.ROBOT_ARM_X, self.ROBOT_ARM_Y), False)
                    comp_thread.start()
                    comp_thread.join()  # wait while computing is being done
                    alpha = comp_thread.alpha
                    length = comp_thread.length
                    theta = comp_thread.theta
                    end_eff = comp_thread.end_eff
                    goal = comp_thread.goal

                    print "\nBase rotation angle:"
                    print alpha
                    print "\nPath length:"
                    print length

                    if theta is not None:
                        print "\nTheta:"
                        print theta
                        print "\nEnd-effector position:"
                        print end_eff
                        print "\nGoal position:"
                        print goal

                        print "\nReady to grip: Press 'G' button to grip the object"
                        k = cv.waitKey(0)
                        if k & 0xFF == ord('g'):
                            self.servodrv.initiate(theta, alpha)
                            res = self.servodrv.grip()

                            if res is True:
                                # Computing angles to move object onto the plate
                                comp_thread = ComputingThread((self.PLATE_X, self.PLATE_Y),
                                                              (self.ROBOT_ARM_X, self.ROBOT_ARM_Y), True)
                                comp_thread.start()
                                comp_thread.join()
                                alpha = comp_thread.alpha
                                theta = comp_thread.theta

                                self.servodrv.initiate(theta, alpha)
                                self.servodrv.take_to_plate()

                    cv.waitKey(0)

                cv.imshow('Webcam', frame)
            else:
                break

        cap1.release()
        cap2.release()
        cv.destroyAllWindows()

    def proj_to_world(self, (u, v), index = CAM_1):
        result = []

        mtx = np.loadtxt('mtx_' + str(index), np.float32)
        rvec = np.loadtxt('rvecs_' + str(index), np.float32)
        tvec = np.loadtxt('tvecs_' + str(index), np.float32)
        rmat, _ = cv.Rodrigues(rvec)
        rtr = np.zeros((3, 4), np.float32)
        rtr[:, :3] = rmat
        rtr[:, 3] = tvec.T

        rtrd = np.delete(rtr, 2, 1)
        mtx_t_rtrd = np.dot(mtx, rtrd)
        inv_mtx_t_rtrd = inv(mtx_t_rtrd)
        uv_vect = np.array([u, v, 1.]).T
        res = np.dot(inv_mtx_t_rtrd, uv_vect)
        result.append(res[0]/res[2])  # X
        result.append(res[1]/res[2])  # Y

        print result

        # Adjusting solution
        cam = -(np.dot(inv(rmat), tvec))

        dx = result[0] - cam[0]
        dy = result[1] - cam[1]
        l = np.sqrt(dx**2 + dy**2)
        dl = self.OBJECT_H * l / cam[2]

        beta = np.abs(np.arctan(dy / dx))
        result[0] += (-1) * InvKin.sign(dx) * dl * np.cos(beta)
        result[1] += (-1) * InvKin.sign(dy) * dl * np.sin(beta)

        print result

        # Determining coordinates relatively to the main CAM_1
        if index == self.CAM_2:
            rtr = np.zeros((2, 3), np.float32)
            rtr[0] = [-1.0, 0, self.SYS2_X]
            rtr[1] = [0.0, -1.0, self.SYS2_Y]
            result = np.dot(rtr, np.float32([result[0], result[1], 1.0]))

        return result

    def capture(self):
        index = self.camera_combo.itemData(self.camera_combo.currentIndex())

        print "Starting camera service ..."
        photo_index = -1
        cap = cv.VideoCapture(index)
        cap.set(3, self.FRAME_WIDTH)
        cap.set(4, self.FRAME_HEIGHT)
        cv.namedWindow("Frames", cv.WINDOW_AUTOSIZE)
        while cap.isOpened():
            (ret, frame) = cap.read()
            if ret:
                cv.imshow("Frames", frame)
                k = cv.waitKey(1)
                if k & 0xFF == ord('q'):
                    break
                elif k & 0xFF == ord('c'):
                    photo_index += 1
                    cv.imwrite("frame_" + str(index) + '_' + str(photo_index) + ".png", frame)

        print "Terminating camera service ...\n"
        cap.release()
        cv.destroyAllWindows()

    def intrinsics(self):
        index = self.camera_combo.itemData(self.camera_combo.currentIndex())

        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((self.POINTS_ALC*self.POINTS_ALR, 3), np.float32)
        objp[:, :2] = np.array([[i, j] for i in range(self.POINTS_ALC) for j in range(self.POINTS_ALR)])
        objpoints = []
        imgpoints = []

        images = glob.glob('frame_' + str(index) + '_*.png')
        for fname in images:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            ret, corners = cv.findChessboardCorners(gray, (self.POINTS_ALR, self.POINTS_ALC), None)
            print ret

            if ret:
                objpoints.append(objp)

                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                img = cv.drawChessboardCorners(img, (self.POINTS_ALR, self.POINTS_ALC), corners2, ret)
                cv.imshow('img', img)
                cv.waitKey(500)

        cv.destroyAllWindows()
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        np.savetxt('mtx_' + str(index), mtx)
        np.savetxt('dist_' + str(index), dist)

    def rtrcam(self):
        index = self.camera_combo.itemData(self.camera_combo.currentIndex())

        cap = cv.VideoCapture(index)
        cap.set(3, self.FRAME_WIDTH)
        cap.set(4, self.FRAME_HEIGHT)
        cv.namedWindow('Rotation/translation', cv.WINDOW_AUTOSIZE)
        while cap.isOpened():
            (ret, frame) = cap.read()
            if ret:
                cv.imshow('Rotation/translation', frame)
                k = cv.waitKey(1)
                if k & 0xFF == ord('c'):
                    corners = self.rtrmtx(frame, index, output=True)
                    if corners is not None:
                        np.savez('corners_' + str(index) + '.npz', corners=corners)
                        frame = self.draw_axes(frame,index=index, output=False, from_file=True)
                    cv.imshow('Rotation/translation', frame)
                    k = cv.waitKey(0)
                    if k & 0xFF == ord('q'):
                        break
                elif k & 0xFF == ord('q'):
                    break
        cap.release()
        cv.destroyAllWindows()

    def rtrmtx(self, frame, index = CAM_1, output = False):
        mtx = np.loadtxt('mtx_' + str(index), np.float32)
        dist = np.loadtxt('dist_' + str(index), np.float32)

        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((self.POINTS_ALR*self.POINTS_ALC, 3), np.float32)
        objp[:, :2] = np.array([[i, j] for i in range(self.POINTS_ALC) for j in range(self.POINTS_ALR)])
        objp *= self.CHESS_SQUARE_SIZE

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        (ret, corners) = cv.findChessboardCorners(gray, (self.POINTS_ALR, self.POINTS_ALC), None)
        if ret:
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
            if ret:
                if output:
                    print "Saving into the file rvecs ..."
                    np.savetxt('rvecs_' + str(index), rvecs)
                    print "... Done"
                    print "Saving into the file tvecs ..."
                    np.savetxt('tvecs_' + str(index), tvecs)
                    print "... Done\n"
                else:
                    np.savetxt('rvecs_' + str(index), rvecs)
                    np.savetxt('tvecs_' + str(index), tvecs)
            else:
                corners2 = None
                print "\nUnable to determine rotation/translation"
        else:
            corners2 = None
            print "\nUnable to determine rotation/translation"

        return corners2

    def draw_axes(self, frame, index = CAM_1, output = False, from_file = False, length = 3, grid = False):
        mtx = np.loadtxt('mtx_' + str(index), np.float32)
        dist = np.loadtxt('dist_' + str(index), np.float32)

        if from_file:
            try:
                npzfile = np.load('corners_' + str(index) + '.npz')
                corners = npzfile['corners']
            except IOError:
                print "Unable to draw axes. Save rtr matrix into the file\n"
                return frame
        else:
            corners = self.rtrmtx(frame, index=index, output=output)

        if corners is None:
            print "Chessboard squares are out of view"
            return frame
        rvecs = np.loadtxt('rvecs_' + str(index), np.float32)
        tvecs = np.loadtxt('tvecs_' + str(index), np.float32)

        world_axes = np.float32([[length * self.CHESS_SQUARE_SIZE, 0, 0],
                                 [0, length * self.CHESS_SQUARE_SIZE, 0],
                                 [0, 0, length * self.CHESS_SQUARE_SIZE]]).reshape(-1, 3)
        # Project 3D points to image frame
        imgpts, jac = cv.projectPoints(world_axes, rvecs, tvecs, mtx, dist)

        if grid:
            frame = cv.drawChessboardCorners(frame, (self.POINTS_ALR, self.POINTS_ALC), corners, True)
        # dots, jac = cv.projectPoints(np.float32([[0, 0, 0], [5 * self.CHESS_SQUARE_SIZE,
        #                                                      8 * self.CHESS_SQUARE_SIZE, 0]]).reshape(-1, 3)
        #                              , rvecs, tvecs, mtx, dist)
        # frame = cv.line(frame, tuple(dots[0].ravel()), tuple(dots[1].ravel()), (197, 57, 58), 5)

        if corners is not None:
            corner = tuple(corners[0].ravel())
            frame = cv.line(frame, corner, tuple(imgpts[0].ravel()), self.BLUE, 5)
            frame = cv.line(frame, corner, tuple(imgpts[1].ravel()), self.GREEN, 5)
            frame = cv.line(frame, corner, tuple(imgpts[2].ravel()), self.RED, 5)

        return frame

    def augreal(self):
        index = self.camera_combo.itemData(self.camera_combo.currentIndex())

        cap = cv.VideoCapture(index)
        cap.set(3, self.FRAME_WIDTH)
        cap.set(4, self.FRAME_HEIGHT)
        cv.namedWindow("Augmented reality", cv.WINDOW_AUTOSIZE)
        while cap.isOpened():
            (ret, frame) = cap.read()
            if ret:
                frame = self.draw_axes(frame, index=index, output=False, from_file=False, grid=True)
                cv.imshow("Augmented reality", frame)
                k = cv.waitKey(1)
                if k & 0xFF == ord('q'):
                    break
        cap.release()
        cv.destroyAllWindows()

    def draw_lines(self, frame, (X, Y), (u, v), index = CAM_1, thickness = 2):
        mtx = np.loadtxt('mtx_' + str(index), np.float32)
        dist = np.loadtxt('dist_' + str(index), np.float32)
        rvecs = np.loadtxt('rvecs_' + str(index), np.float32)
        tvecs = np.loadtxt('tvecs_' + str(index), np.float32)

        points = np.float32([[X, 0, 0], [0, Y, 0]])
        imgpts, _ = cv.projectPoints(points, rvecs, tvecs, mtx, dist)
        frame = cv.line(frame, (u, v), tuple(imgpts[0].ravel()), self.BLUE, thickness)
        frame = cv.line(frame, (u, v), tuple(imgpts[1].ravel()), self.GREEN, thickness)

        return frame, imgpts

    def move_servo(self):
        x = float(self.x_edit.text())
        y = float(self.y_edit.text())

        comp_thread = ComputingThread((x, y),
                                      (self.ROBOT_ARM_X, self.ROBOT_ARM_Y), False)
        comp_thread.start()
        comp_thread.join()
        alpha = comp_thread.alpha
        theta = comp_thread.theta

        self.servodrv.initiate(theta, alpha)
        self.servodrv.grip(True)

    def default(self):
        self.srv1_edit.setText('255')
        self.srv2_edit.setText('255')
        self.srv3_edit.setText('255')
        self.srv4_edit.setText('255')
        self.srv5_edit.setText('255')
        self.srv6_edit.setText('255')

    def idle(self):
        self.srv1_edit.setText('254')
        self.srv2_edit.setText('254')
        self.srv3_edit.setText('254')
        self.srv4_edit.setText('254')
        self.srv5_edit.setText('254')
        self.srv6_edit.setText('254')

    def send_package(self):
        cmds = [int(self.srv1_edit.text()),
                int(self.srv2_edit.text()),
                int(self.srv3_edit.text()),
                int(self.srv4_edit.text()),
                int(self.srv5_edit.text()),
                int(self.srv6_edit.text())]

        print cmds
        self.servodrv.send_package(cmds)

    def open_com(self):
        self.servodrv.__init__(self.com_edit)

    def close_com(self):
        try:
            self.servodrv.close_thread()
            self.servodrv.__del__()
        except AttributeError:
            print "Unable to close port " + self.servodrv.COM_PORT
        finally:
            self.qt_app.quit()

    def run(self):
        ''' Run the app and show the main form. '''
        self.show()
        self.qt_app.exec_()

app = RoboPickApp()
app.run()
