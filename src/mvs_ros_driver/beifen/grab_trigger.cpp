#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>

using namespace std;

vector<double> D{-0.07884282709077937, 0.11718432356726514, 0.0002523398508066392, -0.001583073019430997, 0.0};

boost::array<double, 9> K = {616.6341619018385, 0.0, 313.58694268693466, 0.0, 616.7110484017726, 281.00480945969355, 0.0, 0.0, 1.0};

boost::array<double, 12>
    P = {609.157958984375, 0.0, 312.3349415024859, 0.0, 0.0, 609.4998168945312, 280.4931307291372, 0.0, 0.0, 0.0, 1.0, 0.0};

boost::array<double, 9>
    R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerBG8 = 0x0108000B//(17301515)10
};
// unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
int width, height;
image_transport::CameraPublisher pub;
std::vector<PixelFormat> PIXEL_FORMAT = { RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerBG8 };
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[4] = {"Off", "Once", "Continues","Don't set"};
float image_scale = 0.0;
int trigger_enable = 1;
sensor_msgs::CameraInfo camera_info_msg;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
  if (NULL == pstMVDevInfo)
  {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
  {
    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
  {
    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  else
  {
    printf("Not support.\n");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];
  
  if (image_scale < 0.1) image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int nRet;

			
  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
      std::string msg = "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    ROS_INFO_STREAM(msg.c_str());
  } else {
    if(ExposureAutoMode == 2) {
      ROS_WARN_STREAM("Fail to set Exposure Auto Mode to Continues");
    }
    else {
      ROS_INFO_STREAM(msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time");
    }
  }
  
  printf("GainAuto = %d\n",GainAuto);
  
  if (GainAuto != 3) //Don't set
  {
    nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

    if (MV_OK == nRet) {
      std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Gain auto mode");
    }

    if (GainAuto == 0) {
      nRet = MV_CC_SetGain(handle, Gain);
      if (MV_OK == nRet) {
        std::string msg = "Set Gain: " + std::to_string(Gain);
        ROS_INFO_STREAM(msg.c_str());
      } else {
        ROS_ERROR_STREAM("Fail to set Gain");
      }
    }
  }

  if (GammaSlector != -1)
  {
		nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
		if (MV_OK == nRet) {
		  std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
		  ROS_INFO_STREAM(msg.c_str());
		} else {
		  ROS_ERROR_STREAM("Fail to set GammaSlector");
		}
  }
	printf("Gamma = %.6f\n",Gamma);
	
	if (Gamma + 1 > 0.0001)
	{  
		nRet = MV_CC_SetGamma(handle, Gamma);
		if (MV_OK == nRet) {
		  std::string msg = "Set Gamma: " + std::to_string(Gamma);
		  ROS_INFO_STREAM(msg.c_str());
		} else {
		  ROS_ERROR_STREAM("Fail to set Gamma");
		}
	}
	
  //1hz信号触发出图
  nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 10.0);
  if (MV_OK != nRet) {
    ROS_ERROR_STREAM("fail to set AcquisitionFrameRate");
  }
  
  nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", 10);
  if (MV_OK != nRet) {
    ROS_ERROR_STREAM("fail to set AcquisitionBurstFrameCount");
  }

}

void SignalHandler(int signal) {
  if (signal == SIGINT) {  // 捕捉 Ctrl + C 触发的 SIGINT 信号
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true;    // 设置退出标志
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler; // 设置处理函数
  sigemptyset(&sigIntHandler.sa_mask);      // 清空信号屏蔽集
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
 
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {0};
  
  unsigned char* pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  //unsigned char* pDataForBGR = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);

  if (pData == nullptr) {// || pDataForBGR == nullptr) {
    printf("Memory allocation failed!\n");
    if (pData) free(pData);
    //if (pDataForBGR) free(pDataForBGR);
    return nullptr;
  }

  while (!exit_flag && ros::ok()) {
		//cout << "pUser = " << (long long)pUser;
    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, stParam.nCurValue, &stImageInfo, 150);
    if (nRet == MV_OK) {
      
      ros::Time rcv_time;
      if(trigger_enable && pointt != MAP_FAILED && pointt->low != 0)
      {
        // 赋值共享内存中的时间戳给相机帧
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time = ros::Time(time_pc);
      }
      else
      {
        rcv_time = ros::Time::now();
      }
     
      // std::string debug_msg;
      // debug_msg = "GetOneFrame,nFrameNum[" +
      //             std::to_string(stImageInfo.nFrameNum) + "], FrameTime:" +
      //             std::to_string(rcv_time.toSec());
      // ROS_INFO_STREAM(debug_msg.c_str());
			
      /*stConvertParam.nWidth = stImageInfo.nWidth;
      stConvertParam.nHeight = stImageInfo.nHeight;
      stConvertParam.pSrcData = pData;
      stConvertParam.nSrcDataLen = stParam.nCurValue; 
      stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      stConvertParam.pDstBuffer = pDataForBGR;
      stConvertParam.nDstBufferSize = stParam.nCurValue * 3;*/
			// chayan
			//printf("MAX ++++ = %d\n",USHRT_MAX);
			
			//FILE* fp = fopen("/home/fastlivo2/raw_data.bin", "wb");
			//fwrite(pData, 1, stConvertParam.nSrcDataLen, fp);
			//fclose(fp);

			//cout << "pUser = " << (long long)pUser;
			
      /*nRet = MV_CC_ConvertPixelTypeEx(pUser, &stConvertParam);
      if (MV_OK != nRet)
      {
        printf("MV_CC_ConvertPixelTypeEx failed! nRet [%x], skipping this frame\n", nRet);
        continue;
      }*/
      
      cv::Mat bayerimg,srcImage;
      bayerimg = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC1, pData);
      cv::cvtColor(bayerimg, srcImage, cv::COLOR_BayerBG2RGB); // BGGR 转 RGB
      
      //cv::Mat srcImage; 
      //srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);

      // cv::Mat srcImage;
      // srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      

      if (image_scale > 0.0) {
        cv::resize(srcImage, srcImage, cv::Size(srcImage.cols * image_scale, srcImage.rows * image_scale), cv::INTER_LINEAR);
      } else {
        printf("Invalid image_scale: %f. Skipping resize.\n", image_scale);
      }
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
      msg->header.stamp = rcv_time;
      
      camera_info_msg.header.frame_id = msg->header.frame_id;
      camera_info_msg.header.stamp = rcv_time;
      
      pub.publish(*msg, camera_info_msg);
    }
  }

  if (pData) {
    free(pData);
    pData = nullptr;
  }

  /*if (pDataForBGR)
  {
    free(pDataForBGR);
    pDataForBGR = nullptr;
  }*/

  return 0;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mvs_trigger");
  std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;
  ros::Rate loop_rate(10);
  
  camera_info_msg.height = 540;
  camera_info_msg.width = 650;
  camera_info_msg.distortion_model = "plumb_bob";
  camera_info_msg.D = D;
  camera_info_msg.K = K;
  camera_info_msg.P = P;
  camera_info_msg.R = R;
  camera_info_msg.binning_x = 0;
  camera_info_msg.binning_y = 0;


  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  pub = it.advertiseCamera(pub_topic, 1000);
  
  const char *user_name = getlogin();
  std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();

  int fd = open(shared_file_name, O_RDWR);

  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, 0);

  SetupSignalHandler();
 
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    return -1;
  }

  if (stDeviceList.nDeviceNum > 0)
  {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL)
      {
        printf("Device info is NULL for device %d\n", i);
        return -1;
      } 
      PrintDeviceInfo(pDeviceInfo);            
    }  
  } 
  else
  {
    printf("Find No Devices!\n");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) 
  {
    if (expect_serial_number.empty()) 
    {
      ROS_ERROR("Expected serial number is empty!");
      return -1;
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) 
    {
      if (stDeviceList.pDeviceInfo[i] == NULL) 
      {
        printf("Device info is NULL for device %d\n", i);
        continue;
      }
      
      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE)
      {
        serial_number = 
            std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber);
      }
      else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
      {
        serial_number = 
            std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber);
      }
      else
      {
        printf("Unknown device type!\n");
        continue;
      }
      if (serial_number.empty()) 
      {
        printf("Serial number is empty for device %d\n", i);
        continue;
      }
      if (expect_serial_number == serial_number) 
      {
        find_expect_camera = true;
        nIndex = i;
        break;
      }
    }
    if (!find_expect_camera) 
    {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR_STREAM(msg.c_str());
      return -1;
    }
  }
  else
  {
    nIndex = 0;
  }
  
  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet)
  {
    printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  // open device
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet)
  {
    printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
  if (MV_OK != nRet) {
    printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
    return -1;
  }

  // MVCC_INTVALUE stParam;
  // memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  // nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  // if (MV_OK != nRet) {
  //   printf("Get PayloadSize fail\n");
  //   return -1;
  // }
  // g_nPayloadSize = stParam.nCurValue * 3;

  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[PixelFormat]); // BayerRG8 0x01080009 RGB8 0x02180014 BayerRG12Packed 0x010C002B
  if (nRet != MV_OK) {
    printf("Pixel setting can't work.");
    return -1;
  }
	
	/*MVCC_ENUMVALUE pixelType = {0};
	int ret = MV_CC_GetEnumValue(handle, "PixelFormat", &pixelType);
	
	if (MV_OK != ret)
	{
		printf("错了");
	}
	
	MvGvspPixelType currentType = (MvGvspPixelType)pixelType.nCurValue;
	printf("当前像素格式: %x\n",  currentType);  

	printf("支持的像素格式列表:\n");
	for (unsigned int i = 0; i < pixelType.nSupportedNum; i++) {
    if (pixelType.nSupportValue[i] == 0) continue;  // 跳过无效值
    MvGvspPixelType supportType = (MvGvspPixelType)pixelType.nSupportValue[i];
    printf("  - %x\n", supportType);
	}*/
	  
	setParams(handle, params_file);
	
  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    return -1;
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
    return -1;
  }

  ROS_INFO("Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    printf("Start Grabbing fail.\n");
    return -1;
  }

  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    printf("thread create failed.ret = %d\n", nRet);
    return -1;
  }

  while (!exit_flag && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO_STREAM("Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  munmap(pointt, sizeof(time_stamp));

  return 0;
}


