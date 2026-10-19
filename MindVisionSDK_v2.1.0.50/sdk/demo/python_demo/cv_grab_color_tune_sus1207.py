#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform

# SDK 2.1.0.50: 初始化后再进行设备枚举
sdk_rc = mvsdk.CameraSdkInit(1)
print("CameraSdkInit:", sdk_rc, flush=True)
if sdk_rc != 0:
    raise RuntimeError(f"CameraSdkInit failed: {sdk_rc}")

def main_loop():
	# 枚举相机
	DevList = mvsdk.CameraEnumerateDevice()
	nDev = len(DevList)
	if nDev < 1:
		print("No camera was found!")
		return

	for i, DevInfo in enumerate(DevList):
		print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
	i = 0 if nDev == 1 else int(input("Select camera: "))
	DevInfo = DevList[i]
	print(DevInfo)

	# 打开相机
	hCamera = 0
	try:
		hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
	except mvsdk.CameraException as e:
		print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
		return

	# 获取相机特性描述
	cap = mvsdk.CameraGetCapability(hCamera)

	# 判断是黑白相机还是彩色相机
	monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

	# 黑白相机让ISP直接输出MONO数据，而不是扩展成R=G=B的24位灰度
	if monoCamera:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
	else:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

	# 相机模式切换成连续采集
	mvsdk.CameraSetTriggerMode(hCamera, 0)

	# 自动曝光与自动白平衡（避免原 Demo 固定 30ms 欠曝）
	# 根据 SDK 声明的能力开启自动曝光/白平衡，避免固定 30ms 导致欠曝
	if cap.sIspCapacity.bAutoExposure:
		rc = mvsdk.CameraSetAeState(hCamera, 1)
		print("AutoExposure rc:", rc)
	else:
		print("AutoExposure not supported, keep manual 30ms")
		mvsdk.CameraSetAeState(hCamera, 0)
		mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
	if not monoCamera:
		if cap.sIspCapacity.bAutoWb:
			rc = mvsdk.CameraSetWbMode(hCamera, 1)
			print("AutoWhiteBalance rc:", rc)
		else:
			print("AutoWhiteBalance not supported")
		print("Saturation before:", mvsdk.CameraGetSaturation(hCamera))
		if cap.sSaturationRange.iMin <= 100 <= cap.sSaturationRange.iMax:
			rc = mvsdk.CameraSetSaturation(hCamera, 100)
			print("Set neutral saturation 100 rc:", rc)
	print("AE enabled:", mvsdk.CameraGetAeState(hCamera))
	if not monoCamera:
		print("WB enabled:", mvsdk.CameraGetWbMode(hCamera))


	# 让SDK内部取图线程开始工作
	mvsdk.CameraPlay(hCamera)

	# 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
	FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

	# 分配RGB buffer，用来存放ISP输出的图像
	# 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
	pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

	frame_count = 0
	# Initial controls are only read; camera parameter changes are applied on key presses.
	initial_rgb = mvsdk.CameraGetGain(hCamera)
	initial_sat = mvsdk.CameraGetSaturation(hCamera)
	initial_ae = mvsdk.CameraGetAeTarget(hCamera)
	print("RGB gain (R,G,B):", initial_rgb, "range:",
	      (cap.sRgbGainRange.iRGainMin, cap.sRgbGainRange.iRGainMax),
	      (cap.sRgbGainRange.iGGainMin, cap.sRgbGainRange.iGGainMax),
	      (cap.sRgbGainRange.iBGainMin, cap.sRgbGainRange.iBGainMax), flush=True)
	print("One-shot WB supported:", bool(cap.sIspCapacity.bWbOnce), flush=True)
	print("Controls: w=one-shot WB, r/f=R +/-5, g/h=G +/-5, b/n=B +/-5,"
	      " u/j=saturation +/-10, ]/[=AE target +/-5, p=print, x=reset, q=quit", flush=True)

	def change_rgb(channel, step):
		values = list(mvsdk.CameraGetGain(hCamera))
		ranges = [
			(cap.sRgbGainRange.iRGainMin, cap.sRgbGainRange.iRGainMax),
			(cap.sRgbGainRange.iGGainMin, cap.sRgbGainRange.iGGainMax),
			(cap.sRgbGainRange.iBGainMin, cap.sRgbGainRange.iBGainMax),
		]
		lo, hi = ranges[channel]
		values[channel] = max(lo, min(hi, values[channel] + step))
		rc = mvsdk.CameraSetGain(hCamera, *values)
		print("CameraSetGain rc:", rc, "readback R,G,B:", mvsdk.CameraGetGain(hCamera), flush=True)

	while True:
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
		elif key == ord('w'):
			if cap.sIspCapacity.bWbOnce:
				rc = mvsdk.CameraSetWbMode(hCamera, 0)
				print("Manual WB mode rc:", rc, flush=True)
				if rc == 0:
					rc = mvsdk.CameraSetOnceWB(hCamera)
					print("One-shot WB rc:", rc, flush=True)
			else:
				print("One-shot WB is not supported; use manual RGB controls", flush=True)
		elif key in (ord('r'), ord('f'), ord('g'), ord('h'), ord('b'), ord('n')):
			channel, delta = {
				ord('r'): (0, +5), ord('f'): (0, -5),
				ord('g'): (1, +5), ord('h'): (1, -5),
				ord('b'): (2, +5), ord('n'): (2, -5),
			}[key]
			change_rgb(channel, delta)
		elif key in (ord('u'), ord('j')):
			s = mvsdk.CameraGetSaturation(hCamera) + (10 if key == ord('u') else -10)
			s = max(cap.sSaturationRange.iMin, min(cap.sSaturationRange.iMax, s))
			rc = mvsdk.CameraSetSaturation(hCamera, s)
			print("Saturation rc:", rc, "readback:", mvsdk.CameraGetSaturation(hCamera), flush=True)
		elif key in (ord(']'), ord('[')):
			target = mvsdk.CameraGetAeTarget(hCamera) + (5 if key == ord(']') else -5)
			target = max(cap.sExposeDesc.uiTargetMin, min(cap.sExposeDesc.uiTargetMax, target))
			rc = mvsdk.CameraSetAeTarget(hCamera, target)
			print("AE target rc:", rc, "readback:", mvsdk.CameraGetAeTarget(hCamera), flush=True)
		elif key == ord('p'):
			print("RGB:", mvsdk.CameraGetGain(hCamera), "Sat:", mvsdk.CameraGetSaturation(hCamera),
			      "AE target:", mvsdk.CameraGetAeTarget(hCamera), flush=True)
		elif key == ord('x'):
			print("Reset RGB/Saturation/AE target rc:",
			      mvsdk.CameraSetGain(hCamera, *initial_rgb),
			      mvsdk.CameraSetSaturation(hCamera, initial_sat),
			      mvsdk.CameraSetAeTarget(hCamera, initial_ae), flush=True)
		# 从相机取一帧图片
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

			frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
			cv2.imshow("Press q to end", frame)
			frame_count += 1
			if frame_count % 30 == 0:
				print("Exposure_us:", FrameHead.uiExpTime,
				      "AnalogGain:", FrameHead.fAnalogGain,
				      "Saturation:", FrameHead.iSaturation,
				      "BGR_mean:", frame.mean(axis=(0, 1)).round(1).tolist(), flush=True)
			
		except mvsdk.CameraException as e:
			if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
				print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )

	# 关闭相机
	mvsdk.CameraUnInit(hCamera)

	# 释放帧缓存
	mvsdk.CameraAlignFree(pFrameBuffer)

def main():
	try:
		main_loop()
	finally:
		cv2.destroyAllWindows()

main()
