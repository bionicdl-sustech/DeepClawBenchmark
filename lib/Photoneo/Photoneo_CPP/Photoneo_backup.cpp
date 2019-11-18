#include "PhoXi.h"
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#if defined(_WIN32)
#include <windows.h>
#elif defined (__linux__)
#include <unistd.h>
#endif

#if defined(_WIN32)
#define LOCAL_CROSS_SLEEP(Millis) Sleep(Millis)
#elif defined (__linux__) || defined(__APPLE__)
#define LOCAL_CROSS_SLEEP(Millis) usleep(Millis * 1000)
#endif

//The whole api is in namespace pho (Photoneo) :: api
class PhoXiExamples {
  private:
    template<class T>
    bool ReadLine(T &Output) const {
        std::string Input;
        std::getline(std::cin, Input);
        std::stringstream InputSteam(Input);
        if (InputSteam >> Output) {
            return true;
        } else {
            return false;
        }
    }
    bool ReadLine(std::string &Output) const {
        std::getline(std::cin, Output);
        return true;
    }
  public:
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList;
    pho::api::PPhoXi PhoXiDevice;
    pho::api::PhoXiFactory Factory;
    pho::api::PFrame SampleFrame;
    void ConnectPhoXiDevice() {
	while(true){
	    ConnectFirstAttachedPhoXiDevice();
            if (PhoXiDevice && PhoXiDevice->isConnected()) break;
	}
    }
    void ConnectFirstAttachedPhoXiDevice() {
        PhoXiDevice = Factory.CreateAndConnectFirstAttached();
        if (PhoXiDevice) {
            std::cout << "Connection to the device " << (std::string) PhoXiDevice->HardwareIdentification
                      << " was Successful!" << std::endl;
        } else {
            std::cout << "There is no attached device, or the device is not ready!" << std::endl;
        }
    }
    void SoftwareTrigger(char* timestamp, char* state) {
        //Check if the device is connected
        if (PhoXiDevice && PhoXiDevice->isConnected()) {
            //If it is not in Software trigger mode, we need to switch the modes
            if (PhoXiDevice->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
                std::cout << "Device is not in Software trigger mode" << std::endl;
                if (PhoXiDevice->isAcquiring()) {
                    std::cout << "Stopping acquisition" << std::endl;
                    //If the device is in Acquisition mode, we need to stop the acquisition
                    if (!PhoXiDevice->StopAcquisition()) {
                        throw std::runtime_error("Error in StopAcquistion");
                    }
                }
                std::cout << "Switching to Software trigger mode " << std::endl;
                //Switching the mode is as easy as assigning of a value, it will call the appropriate calls in the background
                PhoXiDevice->TriggerMode = pho::api::PhoXiTriggerMode::Software;
                //Just check if did everything run smoothly
                if (!PhoXiDevice->TriggerMode.isLastOperationSuccessful()) throw std::runtime_error(PhoXiDevice->TriggerMode.GetLastErrorMessage().c_str());
            }
            //Start the device acquisition, if necessary
            if (!PhoXiDevice->isAcquiring()) {
                if (!PhoXiDevice->StartAcquisition()) {
                    throw std::runtime_error("Error in StartAcquisition");
                }
            }
            //We can clear the current Acquisition buffer -- This will not clear Frames that arrives to the PC after the Clear command is performed
            int ClearedFrames = PhoXiDevice->ClearBuffer();
            std::cout << ClearedFrames << " frames were cleared from the cyclic buffer" << std::endl;

            //While we checked the state of the StartAcquisition call, this check is not necessary, but it is a good practice
            if (PhoXiDevice->isAcquiring()) {
                for (std::size_t i = 0; i < 1; i++) {
                    std::cout << "Triggering the " << i << "-th frame" << std::endl;
                    int FrameID =
                        PhoXiDevice->TriggerFrame(/*If false is passed here, the device will reject the frame if it is not ready to be triggered, if true us supplied, it will wait for the trigger*/);
                    if (FrameID < 0) {
                        //If negative number is returned trigger was unsuccessful
                        std::cout << "Trigger was unsuccessful!" << std::endl;
                        continue;
                    } else {
                        std::cout << "Frame was triggered, Frame Id: " << FrameID << std::endl;
                    }
                    std::cout << "Waiting for frame " << i << std::endl;
                    //Wait for a frame with specific FrameID. There is a possibility, that frame triggered before the trigger will arrive after the trigger call, and will be retrieved before requested frame
                    //  Because of this, the TriggerFrame call returns the requested frame ID, so it can than be retrieved from the Frame structure. This call is doing that internally in background
                    pho::api::PFrame Frame =
                        PhoXiDevice->GetSpecificFrame(FrameID/*, You can specify Timeout here - default is the Timeout stored in Timeout Feature -> Infinity by default*/);
                    if (Frame) {
                        std::cout << "Frame retrieved" << std::endl;
                        std::cout << "  Frame params: " << std::endl;
                        std::cout << "    Frame Index: " << Frame->Info.FrameIndex << std::endl;
                        std::cout << "    Frame Timestamp: " << Frame->Info.FrameTimestamp << std::endl;
                        std::cout << "    Frame Duration: " << Frame->Info.FrameDuration << std::endl;
                        std::cout << "    Frame Resolution: " << Frame->GetResolution().Width << " x "
                                  << Frame->GetResolution().Height << std::endl;
                        std::cout << "    Sensor Position: " << Frame->Info.SensorPosition.x << "; "
                                  << Frame->Info.SensorPosition.y << "; " << Frame->Info.SensorPosition.z << std::endl;
                        if (!Frame->Empty()) {
                            std::cout << "  Frame data: " << std::endl;
                            if (!Frame->PointCloud.Empty()) {
                                std::cout << "    PointCloud: " << Frame->PointCloud.Size.Width << " x "
                                          << Frame->PointCloud.Size.Height << " Type: "
                                          << Frame->PointCloud.GetElementName() << std::endl;
                            }
                            if (!Frame->NormalMap.Empty()) {
                                std::cout << "    NormalMap: " << Frame->NormalMap.Size.Width << " x "
                                          << Frame->NormalMap.Size.Height << " Type: "
                                          << Frame->NormalMap.GetElementName() << std::endl;
                            }
                            if (!Frame->DepthMap.Empty()) {
                                std::cout << "    DepthMap: " << Frame->DepthMap.Size.Width << " x "
                                          << Frame->DepthMap.Size.Height << " Type: "
                                          << Frame->DepthMap.GetElementName() << std::endl;
                            }
                            if (!Frame->ConfidenceMap.Empty()) {
                                std::cout << "    ConfidenceMap: " << Frame->ConfidenceMap.Size.Width << " x "
                                          << Frame->ConfidenceMap.Size.Height << " Type: "
                                          << Frame->ConfidenceMap.GetElementName() << std::endl;
                            }
                            if (!Frame->Texture.Empty()) {
                                std::cout << "    Texture: " << Frame->Texture.Size.Width << " x "
                                          << Frame->Texture.Size.Height << " Type: " << Frame->Texture.GetElementName()
                                          << std::endl;
                            }
			    DataHandlingExample(Frame,timestamp,state);
                        } else {
                            std::cout << "Frame is empty.";
                        }
                    } else {
                        std::cout << "Failed to retrieve the frame!";
                    }
                }
            }
        }
    }
    void DataHandlingExample(pho::api::PFrame SampleFrame, char* timestamp, char* state) {
	//Check if we have SampleFrame Data
        if (SampleFrame && !SampleFrame->Empty()) {
            //We will count the number of measured points
            if (!SampleFrame->PointCloud.Empty()) {
                int MeasuredPoints = 0;
                pho::api::Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
                for (int y = 0; y < SampleFrame->PointCloud.Size.Height; y++) {
                    for (int x = 0; x < SampleFrame->PointCloud.Size.Width; x++) {
                        if (SampleFrame->PointCloud[y][x] != ZeroPoint) {
                            MeasuredPoints++;
                        }
                    }
                }
                std::cout << "Your sample Point cloud has " << MeasuredPoints << " measured points." << std::endl;
                pho::api::Point3_32f *RawPointer = SampleFrame->PointCloud.GetDataPtr();
                float *MyLocalCopy = new float[SampleFrame->PointCloud.GetElementsCount() * 3];
                memcpy(MyLocalCopy, RawPointer, SampleFrame->PointCloud.GetDataSize());
                //Data are organized as a matrix of X, Y, Z floats, see the documentation for all other types
                delete[] MyLocalCopy;
                //Data from SampleFrame, or all other frames that are returned by the device are copied from the Cyclic buffer and will remain in the memory until the Frame will go out of scope
                //You can specifically call SampleFrame->PointCloud.Clear() to release some of the data
            }
            //You can store the Frame as a ply structure
	    std::string filename=strcat(timestamp,state);
	    std::string Filename=filename+".ply";
            SampleFrame->SaveAsPly(Filename/*, You have multiple storing options*/);
	    std::cout<<"save to "<<Filename<<std::endl;
            //If you want OpenCV support, you need to link appropriate libraries and add OpenCV include directory
            //To add the support, add #define PHOXI_OPENCV_SUPPORT before include of PhoXi include files
#ifdef PHOXI_OPENCV_SUPPORT
            if (!SampleFrame->PointCloud.Empty()) {
                cv::Mat PointCloudMat;
                if (SampleFrame->PointCloud.ConvertTo(PointCloudMat)) {
                    cv::Point3f MiddlePoint = PointCloudMat.at<cv::Point3f>(PointCloudMat.rows, PointCloudMat.cols);
                    std::cout << "Middle point: " << MiddlePoint.x << "; " << MiddlePoint.y << "; " << MiddlePoint.z;
                }
            }
#endif
            //If you want PCL support, you need to link appropriate libraries and add PCL include directory
            //To add the support, add #define PHOXI_PCL_SUPPORT before include of PhoXi include files
#ifdef PHOXI_PCL_SUPPORT
            //The PCL convert will convert the appropriate data into the pcl PointCloud based on the Point Cloud type
            pcl::PointCloud<pcl::PointXYZRGBNormal> MyPCLCloud;
            SampleFrame->ConvertTo(MyPCLCloud);
#endif
        }
    }
    void Disconnect() {
        //The whole API is designed on C++ standards, using smart pointers and constructor/destructor logic
        //All resources will be closed automatically, but the device state will not be affected -> it will remain connected in PhoXi Control and if in freerun, it will remain Scanning
        //To Stop the device, just
        return;
    }
    PhoXiExamples(char* timestamp, char* state) {
        try {
            ConnectPhoXiDevice();
            SoftwareTrigger(timestamp,state);
            Disconnect();
        } catch (std::runtime_error &InternalException) {
            std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    PhoXiExamples Example(argv[1], argv[2]);
    return 0;
}

