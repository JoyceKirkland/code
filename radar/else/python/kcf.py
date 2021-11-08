#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
# import torch
from torch._C import Size


class MessageItem(object):
    # 用于封装信息的类,包含图片和其他信息
    def __init__(self,frame,message):
        self._frame = frame
        self._message = message
 
    def getFrame(self):
        # 图片信息
        return self._frame
 
    def getMessage(self):
        #文字信息,json格式
        return self._message

class Tracker(object):
    '''
    追踪者模块,用于追踪指定目标
    '''
 
    def __init__(self, tracker_type="BOOSTING", draw_coord=True):
        '''
        初始化追踪器种类
        '''
        # 获得opencv版本
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        self.tracker_type = tracker_type
        self.isWorking = False
        self.draw_coord = draw_coord
        # 构造追踪器
        if int(major_ver) < 3:
            self.tracker = cv2.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING':
                self.tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                self.tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                self.tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                self.tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                self.tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                self.tracker = cv2.TrackerGOTURN_create()
 
    def initWorking(self, frame, box):
        '''
        追踪器工作初始化
        frame:初始化追踪画面
        box:追踪的区域
        '''
        if not self.tracker:
            raise Exception("追踪器未初始化")
        status = self.tracker.init(frame, box)
        if not status:
            raise Exception("追踪器工作初始化失败")
        self.coord = box
        self.isWorking = True
 
    def track(self, frame):
        '''
        开启追踪
        '''
        message = None
        if self.isWorking:
            status, self.coord = self.tracker.update(frame)
            if status:
                message = {"coord": [((int(self.coord[0]), int(self.coord[1])),
                                      (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3])))]}
                if self.draw_coord:
                    p1 = (int(self.coord[0]), int(self.coord[1]))
                    p2 = (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3]))
                    cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                    message['msg'] = "is tracking"
        return MessageItem(frame, message)
		

class Camera(object):
	def __init__(self, DevInfo):
		super(Camera, self).__init__()
		self.DevInfo = DevInfo
		self.hCamera = 0
		self.cap = None
		self.pFrameBuffer = 0

	def open(self):
		if self.hCamera > 0:
			return True

		# 打开相机
		hCamera = 0
		try:
			hCamera = mvsdk.CameraInit(self.DevInfo, -1, -1)
		except mvsdk.CameraException as e:
			print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
			return False

		# 获取相机特性描述
		cap = mvsdk.CameraGetCapability(hCamera)

		# 判断是黑白相机还是彩色相机
		monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

		# 黑白相机让ISP直接输出MONO数据，而不是扩展成R=G=B的24位灰度
		if monoCamera:
			mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
		else:
			mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

		# 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
		FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

		# 分配RGB buffer，用来存放ISP输出的图像
		# 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
		pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

		# 相机模式切换成连续采集
		# mvsdk.CameraSetTriggerMode(hCamera, 0)
		mvsdk.CameraSetGain(hCamera, 167, 119, 100)

		# 手动曝光，曝光时间30ms
		mvsdk.CameraSetAeState(hCamera, 0)
		mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

		# 让SDK内部取图线程开始工作
		mvsdk.CameraPlay(hCamera)

		self.hCamera = hCamera
		self.pFrameBuffer = pFrameBuffer
		self.cap = cap
		return True

	def close(self):
		if self.hCamera > 0:
			mvsdk.CameraUnInit(self.hCamera)
			self.hCamera = 0

		mvsdk.CameraAlignFree(self.pFrameBuffer)
		self.pFrameBuffer = 0

	def grab(self):
		# 从相机取一帧图片
		hCamera = self.hCamera
		pFrameBuffer = self.pFrameBuffer
		try:
			pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
			mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
			mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

			# windows下取到的图像数据是上下颠倒的，以BMP格式存放。转换成opencv则需要上下翻转成正的
			# linux下直接输出正的，不需要上下翻转
			if platform.system() == "Windows":
				mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
			
			# 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
			# 把pFrameBuffer转换成opencv的图像格式以进行后续算法处理
			frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
			frame = np.frombuffer(frame_data, dtype=np.uint8)
			frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )
			return frame
		except mvsdk.CameraException as e:
			if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
				print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )
			return None

def main_loop():
	# 枚举相机
	DevList = mvsdk.CameraEnumerateDevice()
	nDev = len(DevList)
	if nDev < 1:
		print("No camera was found!")
		return

	for i, DevInfo in enumerate(DevList):
		print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))

	cams = []
	for i in range(len(DevList)):
		cam = Camera(DevList[i])
		if cam.open():
			cams.append(cam)

	font=cv2.FONT_HERSHEY_SIMPLEX
	# frame = cam.grab()
	while True:
		# for cam in cams:
		frame = cam.grab()
		frame1= frame
		if frame is None:
			# frame = cam.grab()
			print('error')
			quit()                
		_key = cv2.waitKey(30) & 0xFF
		if(_key==ord('q')):
			quit()
		if(_key==ord('y')):
			print('yy')
			break
		# cv2.imshow("{} pick frame".format(cam.DevInfo.GetFriendlyName()), frame)
		# cv2.imshow("frame", frame)
		cv2.imshow("pick frame", frame1)		

	# cv2.destroyWindow("pick frame")
	gROI = cv2.selectROI("ROI frame",frame1,False)
	if (not gROI):
		print("空框选，退出")
		quit()
	print(type(gROI))
	print(gROI)
	gTracker = Tracker(tracker_type="KCF")
	gTracker.initWorking(frame1,gROI)

	while (cv2.waitKey(1) & 0xFF) != ord('q'):
		# for cam in cams:
		if frame is not None:
			frame = cam.grab()
			
			_item=gTracker.track(frame1)
			cv2.imshow("track result",_item.getFrame())

			if _item.getMessage():
				print(_item.getMessage())
			else:
				print("丢失，重新使用初始ROI开始")
				gTracker=Tracker(tracker_type="KCF")
				gTracker.initWorking(frame1,gROI)

			_key=cv2.waitKey(1)&0xFF
			if(_key == ord('q')) | (_key == 27):
				break
			if(_key == ord('r')):
				print("用户请求用初始ROI")
				gTracker=Tracker(tracker_type="KCF")
				gTracker.initWorking(frame1,gROI)
			# frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
			# cv2.imshow("{} Press q to end".format(cam.DevInfo.GetFriendlyName()), frame)

		else:
			print("捕获帧失败")
			quit()

	for cam in cams:
		cam.close()

def main():
	try:
		main_loop()
	finally:
		cv2.destroyAllWindows()

main()
