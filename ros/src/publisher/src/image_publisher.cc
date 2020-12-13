//spinnaker
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>


#include <ros/ros.h>

#include <util/image_util.h>

#include <iostream>
#include <popl.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

struct PubrCompo{
    CameraPtr *pCam;
    image_transport::Publisher *publisher;
};

// This function acquires and saves 10 images from a camera.
void AcquiAndPubImages(PubrCompo arg)
{
    CameraPtr pCam = *(arg.pCam);
    image_transport::Publisher publisher=*(arg.publisher);

    try
    {
        // Retrieve TL device nodemap
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        // Retrieve device serial number for filename
        CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

        std::string serialNumber = "";

        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            serialNumber = ptrStringSerial->GetValue();
        }

        cout << endl
             << "[" << serialNumber << "] "
             << "*** IMAGE ACQUISITION THREAD STARTING"
             << " ***" << endl
             << endl;

        cout<<(pCam!=NULL)<<endl;

        // Initialize camera
        pCam->Init();
        pCam->PixelFormat.SetValue(PixelFormat_BayerGB8);//runqiu: get raw images

        std::cout << "Set hardwareTrigger" << std::endl;
        pCam->TriggerMode.SetValue(TriggerMode_On);
        pCam->TriggerSource.SetValue(TriggerSource_Line0);

        //continuous exposure time
        pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);

        // Limit exposure time
        //   pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);
        //   pCam->AutoExposureExposureTimeUpperLimit.SetValue(20000);
        // Fix exposure time
        //pCam->ExposureAuto.SetValue(ExposureAuto_Off);
        //pCam->ExposureTime.SetValue(expoTime);

#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;

        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(pCam, pCam->GetNodeMap(), pCam->GetTLDeviceNodeMap()) != 0)
        {
            return (void*)0;

        }

        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

        // Set acquisition mode to continuous
        CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << serialNumber
                 << "). Aborting..." << endl
                 << endl;
            return;

        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to Continuous (entry 'Continuous' retrieval " << serialNumber
                 << "). Aborting..." << endl
                 << endl;
            return;

        }

        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        cout << "[" << serialNumber << "] "
             << "Acquisition mode set to continusFrame..." << endl;

        // Begin acquiring images
        pCam->BeginAcquisition();

        cout << "[" << serialNumber << "] "
             << "Started acquiring images..." << endl;

        //
        // Retrieve, convert, and save images for each camera
        //
        //const unsigned int k_numImages = 1;//runqiu

        //cout << endl;

        //for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
        while(ros::ok())
        {
            try
            {
                // Retrieve next received image and ensure image completion
                ImagePtr pResultImage = pCam->GetNextImage();

                if (pResultImage->IsIncomplete())
                {
                    cout << "[" << serialNumber << "] "
                         << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl
                         << endl;cout<<"get one frame"<<endl;
                }
                else
                {
                    // // Convert image to mono 8
                    //ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
                    cv::Mat img_;
                    int width = pResultImage->GetWidth();
                    int height = pResultImage->GetHeight();
                    cv::Mat img(height, width, CV_8UC1, pResultImage->GetData());
                    cv::cvtColor(img, img_, cv::COLOR_BayerGR2BGR);

                    sensor_msgs::ImagePtr msg;
                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();
                    (*msg).header.stamp = ros::Time::now();
                    publisher.publish(msg);
                }

                // Release image
                pResultImage->Release();

                //cout << endl;
            }
            catch (Spinnaker::Exception& e)
            {
                cout << "[" << serialNumber << "] "
                     << "Error: " << e.what() << endl;
            }
        }

        // End acquisition
        pCam->EndAcquisition();

        // Deinitialize camera
        pCam->DeInit();

        return;

    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "image_publisher");

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current spinnaker version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
         << endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;

        return -1;
    }

    // initialize this node
    ros::NodeHandle nh;
    image_transport::ImageTransport itl(nh);
    image_transport::ImageTransport itr(nh);
    image_transport::Publisher *publisher=new image_transport::Publisher[2];
    publisher[0]= itl.advertise("imagel", 1);
    publisher[1] = itr.advertise("imager", 1);

    //ros::Rate pub_rate(5);

    // Retrieve camera list size
    unsigned int camListSize = 0;
    camListSize = camList.GetSize();

    // Create an array of CameraPtrs. This array maintenances smart pointer's reference
    // count when CameraPtr is passed into grab thread as void pointer

    // Create an array of handles
    CameraPtr* pCamList = new CameraPtr[camListSize];
    PubrCompo* pubrcom=new PubrCompo[camListSize];
    std::thread myThreads[camListSize];

    try
    {
        for (unsigned int i = 0; i < camListSize; i++)
        {
            // Select camera
            pCamList[i] = camList.GetByIndex(i);

            // Start thread
            pubrcom[i].pCam=&pCamList[i];
            pubrcom[i].publisher=&publisher[i];
            myThreads[i] = std::thread(AcquiAndPubImages, pubrcom[i]);
            cout << "Started one thread" << endl;
        }
        for (unsigned int i = 0; i < camListSize; i++)
        {
            // Wait for all threads to finish
            myThreads[i].join();
        }

        // Clear CameraPtr array
        for (unsigned int i = 0; i < camListSize; i++)
        {
            pCamList[i] = 0;
        }

        // Delete array pointer
        delete[] pCamList;

    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
    }

    //ros::spinOnce();
    //pub_rate.sleep();

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    return EXIT_SUCCESS;
}
