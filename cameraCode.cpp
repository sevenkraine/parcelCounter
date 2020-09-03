#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include "/opt/MVS/include/MvCameraControl.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/dnn.hpp>


bool g_bExit = false;

cv::CascadeClassifier face_cascade;

void findAndContour(cv::Mat img1, cv::Mat img);
cv::String face_cascade_name = "../../../../../../../usr/local/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml";

// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
    {
        fprintf(stderr, "\nPress enter to exit.\n");
    };

    while (getchar() != '\n')
        fprintf(stderr, "\nPress enter to exit.\n");
    g_bExit = true;

    sleep(1);
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
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

        //print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

static void *WorkThread1(void *pUser)
{
    if( !face_cascade.load( face_cascade_name ) )
    {
        printf("can't open the face cascade\n") ;

    };
    int nRet = MV_OK;
    while (1)
    {
        nRet = MV_CC_SetCommandValue(pUser, "TriggerSoftware");
        if (MV_OK != nRet)
        {
            printf("failed in TriggerSoftware[%x]\n", nRet);
        }
        cv::Mat img;
        img = cv::imread("../image.jpg", cv::IMREAD_COLOR);
        if (img.empty())
        {
            printf("couldn't read it\n");
            continue;
        }
        char k = cv::waitKey(1);
        cv::Mat imGray;
        cv::Mat Canny;
        cv::cvtColor(img, imGray, cv::COLOR_BGR2GRAY);
        //cv::Canny(imGray,Canny,100,200);
        cv::equalizeHist(imGray, imGray);
    
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(Canny, contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
        findAndContour(imGray, img);
        cv::imshow("hello", img);
        
        system("./../client/build/client -m mhsrgb_graphdef -s MHS ../empty.png");

        if (g_bExit)
        break;
    }
}


static void *WorkThread(void *pUser)
{
    int nRet = MV_OK;
    unsigned char *pDataForSaveImage = NULL;
    // Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;

    while (1)
    {
        if (g_bExit)
        {
            break;
        }

        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    sleep(1);

        if (nRet == MV_OK)
        {
            printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
                   stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
            

            pDataForSaveImage = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
            if (NULL == pDataForSaveImage)
            {
                break;
            }

            // fill in the parameters of save image
            MV_SAVE_IMAGE_PARAM_EX stSaveParam;
            memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
            // Top to bottom are：
            stSaveParam.enImageType = MV_Image_Jpeg;
            stSaveParam.enPixelType = stImageInfo.enPixelType;
            stSaveParam.nBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
            stSaveParam.nWidth = stImageInfo.nWidth;
            stSaveParam.nHeight = stImageInfo.nHeight;
            stSaveParam.pData = pData;
            stSaveParam.nDataLen = stImageInfo.nFrameLen;
            stSaveParam.pImageBuffer = pDataForSaveImage;
            stSaveParam.nJpgQuality = 80;

            nRet = MV_CC_SaveImageEx2(pUser, &stSaveParam);
            if (MV_OK != nRet)
            {
                printf("failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
                break;
            }

            FILE *fp = fopen("../image.jpg", "wb");
            if (NULL == fp)
            {
                printf("fopen failed\n");
                break;
            }
            fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, fp);
            fclose(fp);
            printf("save image succeed\n");
        }
        else
        {
            printf("No data[%x]\n", nRet);
        }
    }

    free(pData);
    return 0;
}

int main()
{  


    int nRet = MV_OK;

    void *handle = NULL;

    do
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Found No Devices!\n");
            break;
        }

        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;
        scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            break;
        }

        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }
        pthread_t nThreadID1;
        nRet = pthread_create(&nThreadID1, NULL, WorkThread1, handle);

        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            break;
        }

        PressEnterToExit();
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            break;
        }
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("exit\n");
    return 0;
}
void findAndContour(cv::Mat img1, cv::Mat img){

    std::vector<cv::Rect> faces;

    face_cascade.detectMultiScale(img1, faces);
    
    
    //put the rectangle on the faces
    for(size_t i=0; i<faces.size();i++){
              
        cv::rectangle(img,faces[i],cv::Scalar(255,0,0),4,0,0);

    }
}
