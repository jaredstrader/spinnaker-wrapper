#include <spinnaker_wrapper/spinnaker_wrapper.hpp>

using namespace std;

// Constructor
ROS_Spinnaker::ROS_Spinnaker()
{
    // print application build information
    ROS_INFO("Application build date: %s %s", __DATE__, __TIME__);

    // init variables
    pCam_ = NULL;
    timestamp_ = 0;
}

int ROS_Spinnaker::load_cameras()
{
    // Retrieve singleton reference to system object
    system_ = System::GetInstance();

    // Retrieve list of cameras from the system
    camList_ = system_->GetCameras();

    // Get number of cameras
    numCameras_ = camList_.GetSize();
    ROS_INFO("Number of cameras detected: %i", numCameras_);

    // Check if at least one camera is connected
    if (numCameras_ == 0)
    {
        // Clear camera list before releasing system
        camList_.Clear();

        // Release system
        system_->ReleaseInstance();

        cout << "No cameras detected!" << endl;
    
        return 1;
    }

    return 0;   
}

void ROS_Spinnaker::init()
{
    ROS_INFO("Running init()");

    //select camera
    pCam_ = camList_.GetByIndex(0);

    //print device info
    INodeMap & nodeMapTLDevice = pCam_->GetTLDeviceNodeMap();
    PrintDeviceInfo(nodeMapTLDevice);

    //init camera
    try
    {
        pCam_->Init();
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
    }
}

void ROS_Spinnaker::deinit()
{
    ROS_INFO("Running deinit()");
    pCam_->DeInit();

    // Release reference to the camera
    pCam_ = NULL;

    // Clear camera list before releasing system
    camList_.Clear();

    // Release system
    system_->ReleaseInstance(); 
}

void ROS_Spinnaker::begin_acquisition()
{
    ROS_INFO("Begin Acquisition...");
    pCam_->BeginAcquisition();
}

void ROS_Spinnaker::end_acquisition()
{
    ROS_INFO("End Acquisition...");
    pCam_->EndAcquisition();    
}

cv::Mat ROS_Spinnaker::grab_frame()
{
    ImagePtr pResultImage = pCam_->GetNextImage();

    // Check if the Image is complete
    if (pResultImage->IsIncomplete()) {
        
        std::cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "!" << std::endl;

    } else {

        timestamp_ = pResultImage->GetTimeStamp();

        if (frameID_ >= 0) {
            lastFrameID_ = frameID_;
            frameID_ = pResultImage->GetFrameID();
            ROS_WARN_STREAM_COND(frameID_ > lastFrameID_ + 1,"Frames are being skipped!");
        } else {
            frameID_ = pResultImage->GetFrameID();
            ROS_ASSERT_MSG(frameID_ == 0 ,"First frame ID was not zero! Might cause sync issues later...");
        }

    }

    std::cout << "Grabbed frame from camera " << frameID_ << " with timestamp " << timestamp_*1000 << std::endl;

    _image = convert_to_mat(pResultImage);
    pResultImage->Release();

    return _image;
}

void ROS_Spinnaker::set_enum_value(std::string setting, std::string value)
{
    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
    {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (enum retrieval). Aborting...");
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
    {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (entry retrieval). Aborting...");
    }
        
    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();
        
    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);    

    ROS_DEBUG_STREAM(setting << " set to " << value);
}

void ROS_Spinnaker::set_int_value(std::string setting, int value)
{
    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(value);

    ROS_DEBUG_STREAM(setting << " set to " << value);
}

void ROS_Spinnaker::set_float_value(std::string setting, float value)
{
    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (ptr retrieval). Aborting...");
    }
    ptr->SetValue(value);

    ROS_DEBUG_STREAM(setting << " set to " << value);
}

void ROS_Spinnaker::set_bool_value(std::string setting, bool value)
{
    INodeMap & nodeMap = pCam_->GetNodeMap();
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        ROS_FATAL_STREAM("Unable to set " << setting << " to " << value << " (ptr retrieval). Aborting...");
    }
    if (value) ptr->SetValue("True");
        else ptr->SetValue("False");

    ROS_DEBUG_STREAM(setting << " set to " << value);
}

void ROS_Spinnaker::set_buffer_size(int numBuf) {

    INodeMap & sNodeMap = pCam_->GetTLStreamNodeMap();
    CIntegerPtr StreamNode = sNodeMap.GetNode("StreamDefaultBufferCount");
    int64_t bufferCount = StreamNode->GetValue();
    if (!IsAvailable(StreamNode) || !IsWritable(StreamNode)){
        ROS_FATAL_STREAM("Unable to set StreamMode " << "). Aborting...");
        return;
    }
    StreamNode->SetValue(numBuf);
    ROS_DEBUG_STREAM("Set Buf "<<numBuf<<endl);
}

cv::Mat ROS_Spinnaker::convert_to_mat(ImagePtr pImage) 
{

    ImagePtr convertedImage;
    // if (COLOR_)
    convertedImage = pImage->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::NEAREST_NEIGHBOR); //, NEAREST_NEIGHBOR);
    // else
        // convertedImage = pImage->Convert(PixelFormat_Mono8); //, NEAREST_NEIGHBOR);
        
    unsigned int XPadding = convertedImage->GetXPadding();
    unsigned int YPadding = convertedImage->GetYPadding();
    unsigned int rowsize = convertedImage->GetWidth();
    unsigned int colsize = convertedImage->GetHeight();

    //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
    cv::Mat img;
    img = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());

    return img.clone();
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int ROS_Spinnaker::PrintDeviceInfo(INodeMap & nodeMap)
{
    int result = 0;

    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it)
            {
                CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = (CValuePtr)pfeatureNode;
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// // Initialize 
// int ROS_Spinnaker::start()
// {
//     // Retrieve singleton reference to system object
//     system = System::GetInstance();

//     // Retrieve list of cameras from the system
//     camList = system->GetCameras();

//     // Get number of cameras
//     numCameras = camList.GetSize();
//     ROS_INFO("Number of cameras detected: %i", numCameras);

//     // Check if at least one camera is connected
//     if (numCameras == 0)
//     {
//         // Clear camera list before releasing system
//         camList.Clear();

//         // Release system
//         system->ReleaseInstance();

//         cout << "No cameras detected!" << endl;
    
//         return 1;
//     }

//     return 0;
// }

// // Acquire images
// int ROS_Spinnaker::acquire_images()
// {
//     CameraPtr pCam = NULL;

//     // Run example on each camera
//     int result = 0;
//     for (unsigned int i = 0; i < numCameras; i++)
//     {
//         // Select camera
//         pCam = camList.GetByIndex(i);

//         cout << endl << "Running continuous capture for camera " << i << "..." << endl;

//         // Run example
//         result = result | RunSingleCamera(pCam);

//         cout << "Streaming for camera " << i << " complete! Exiting program..." << endl << endl;
//     }  
    
//     // Release reference to the camera
//     pCam = NULL;

//     // Clear camera list before releasing system
//     camList.Clear();

//     // Release system
//     system->ReleaseInstance(); 
// }

// // This function acquires and saves 10 images from a device.  
// int ROS_Spinnaker::AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
// {
//     int result = 0;
    
//     cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
    
//     try
//     {
//         //////// Set acquisition mode to continuous ////////
//         // Retrieve enumeration node from nodemap
//         CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
//         if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
//         {
//             cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
//             return -1;
//         }
        
//         // Retrieve entry node from enumeration node
//         CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
//         if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
//         {
//             cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
//             return -1;
//         }
        
//         // Retrieve integer value from entry node
//         // int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
//         // std::cout << "int64_t acquisitionModeContinuous = " << std::endl;
        
//         // Set integer value from entry node as new value of enumeration node
//         // ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        
//         cout << "Acquisition mode set to continuous..." << endl;
        
//         //////// Begin acquiring images ////////
//         // Note: Image acquisition must be ended when no more images are needed.
//         pCam->BeginAcquisition();

//         cout << "Acquiring images..." << endl;
        
//         //////// Retrieve device serial number for filename ////////
//         gcstring deviceSerialNumber("");
//         CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
//         if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
//         {
//             deviceSerialNumber = ptrStringSerial->GetValue();

//             cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
//         }
//         cout << endl;
        
//         //////// Retrieve, convert, and save images ////////
//         const unsigned int k_numImages = 10;
        
//         for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
//         {
//             try
//             {
//                 //////// Retrieve next received image ////////
//                 ImagePtr pResultImage = pCam->GetNextImage();

//                 //////// Ensure image completion ////////
//                 if (pResultImage->IsIncomplete())
//                 {
                    
//                     cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
//                 }
//                 else
//                 {
//                     //////// Print image information; height and width recorded in pixels ////////
//                     size_t width = pResultImage->GetWidth();
//                     size_t height = pResultImage->GetHeight();
//                     cout << "Grabbed image " << imageCnt << ", width = " << width << ", height = " << height << endl;

//                     //////// Convert image to mono 8 ////////
//                     //ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);

//                     // Create a unique filename
//                     ostringstream filename;
                    
//                     filename << "(DELETE THIS FILE)-Acquisition-";
//                     if (deviceSerialNumber != "")
//                     {
//                             filename << deviceSerialNumber.c_str() << "-";
//                     }
//                     filename << imageCnt << ".jpg";

//                     //////// Save image ////////
//                     //convertedImage->Save(filename.str().c_str());
//                     pResultImage->Save(filename.str().c_str());

//                     cout << "Image saved at " << filename.str() << endl;
//                 }

//                 //////// Release image ////////
//                 pResultImage->Release();

//                 cout << endl;
//             }
//             catch (Spinnaker::Exception &e)
//             {
//                 cout << "Error: " << e.what() << endl;
//                 result = -1;
//             }
//         }
        
//         //////// End acquisition ////////
//         pCam->EndAcquisition();
//     }
//     catch (Spinnaker::Exception &e)
//     {
//         cout << "Error: " << e.what() << endl;
//         result = -1;
//     }
    
//     return result;
// }

// // This function acts as the body of the example; please see NodeMapInfo example 
// // for more in-depth comments on setting up cameras.
// int ROS_Spinnaker::RunSingleCamera(CameraPtr pCam)
// {
//     int result = 0;

//     try
//     {
//         // Retrieve TL device nodemap and print device information
//         INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

//         result = PrintDeviceInfo(nodeMapTLDevice);

//         // Initialize camera
//         pCam->Init();

//         // Retrieve GenICam nodemap
//         INodeMap & nodeMap = pCam->GetNodeMap();

//         // Acquire images
//         result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice);

//         // Deinitialize camera
//         pCam->DeInit();
//     }
//     catch (Spinnaker::Exception &e)
//     {
//         cout << "Error: " << e.what() << endl;
//         result = -1;
//     }

//     return result;
// }