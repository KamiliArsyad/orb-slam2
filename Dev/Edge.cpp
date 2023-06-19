/**
 * This code is made for monocular eurocc dataset. No setting file is needed or used as of right now as we
 * are only ensure that the proposed model is working.
 */
#include <VPIORBNetStream.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <vpi/OpenCVInterop.hpp>

#include <vpi/Array.h>
#include <vpi/Image.h>
#include <vpi/ImageFormat.h>
#include <vpi/Pyramid.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/GaussianPyramid.h>
#include <vpi/algo/ImageFlip.h>
#include <vpi/algo/ORB.h>

#include <bitset>
#include <cstdio>
#include <cstring> // for memset
#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>

// Custom exception class for VPI errors
class VPIException : public std::exception
{
public:
  VPIException(VPIStatus status)
      : msg("VPI error: " + std::string(vpiStatusGetName(status))) {}

  const char *what() const noexcept override
  {
    return msg.c_str();
  }

private:
  std::string msg;
};

// Wrapper function for VPI calls
template <typename Func, typename... Args>
void vpiCall(Func func, Args... args)
{
  VPIStatus status = func(args...);
  if (status != VPI_SUCCESS)
  {
    throw VPIException(status);
  }
}

void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps);
/**
 * First argument: backend (<cpu|cuda>)
 * Second argument: the sequence directory
 */
int main(int argc, char *argv[])
{
  // OpenCV image that will be wrapped by a VPIImage.
  // Define it here so that it's destroyed *after* wrapper is destroyed
  VPIPayload orbPayload = NULL;
  VPIStream stream = NULL;

  int returnValue = 0;

  // Parse parameters
  if (argc != 3)
  {
    throw std::runtime_error(std::string("Usage: ") + argv[0] + " <cpu|cuda> <path_to_image_folder>" + " <path_to_times_file>");
  }

  VPIBackend backend;
  
  if (strcmp(argv[1], "cpu") == 0)
  {
    backend = VPI_BACKEND_CPU;
  }
  else if (strcmp(argv[1], "cuda") == 0)
  {
    backend = VPI_BACKEND_CUDA;
  }
  else
  {
    throw std::runtime_error("Invalid backend");
  }

  std::vector<std::string> vstrImageFilenames;
  std::vector<double> vTimestamps;

  // Load images and timestamps
  LoadImages(argv[2], argv[3], vstrImageFilenames, vTimestamps);

  int numOfFrames = vstrImageFilenames.size();

  if (numOfFrames <= 0)
  {
    throw std::runtime_error("Couldn't load images");
  }

  try
  {
    //      ---------------------
    VPIORBParams orbParams;
    // Create the stream that will be processed in the provided backend
    vpiCall(vpiStreamCreate, backend, &stream);
    vpiCall(vpiInitORBParams, &orbParams);
    orbParams.fastParams.intensityThreshold = 30;
    orbParams.maxFeatures = 1000;
    //      ---------------------

    // Initialize a timer
    cv::TickMeter timer;
    timer.start();

    // Declare VPI objects
    VPIPyramid pyrInput = NULL;
    VPIImage vpiFrame = NULL;
    VPIImage vpiFrameGrayScale = NULL;
    VPIArray keypoints = NULL;
    VPIArray descriptors = NULL;

    cv::Mat frame;
    frame = cv::imread(vstrImageFilenames[0], cv::IMREAD_UNCHANGED);

    if (frame.empty())
    {
      throw std::runtime_error("Couldn't load image");
    }

    vpiCall(vpiImageCreate, frame.cols, frame.rows, VPI_IMAGE_FORMAT_U8, 0, &vpiFrameGrayScale);

    // Setup a worker stream
    ORB_SLAM2::VPIORBNetStream orbStream(9999);

    // Create the output keypoint array.
    vpiCall(vpiArrayCreate,
            orbParams.maxFeatures,
            VPI_ARRAY_TYPE_KEYPOINT_F32,
            backend | VPI_BACKEND_CPU,
            &keypoints);

    // Create the output descriptors array.
    vpiCall(vpiArrayCreate,
            orbParams.maxFeatures,
            VPI_ARRAY_TYPE_BRIEF_DESCRIPTOR,
            backend | VPI_BACKEND_CPU,
            &descriptors);

    // Create the payload for ORB Feature Detector algorithm
    vpiCall(vpiCreateORBFeatureDetector, backend, 10000, &orbPayload);

    // Create the pyramid
    vpiCall(vpiPyramidCreate, frame.cols, frame.rows, VPI_IMAGE_FORMAT_U8, orbParams.pyramidLevels, 0.5,
            backend, &pyrInput);

    // Process each frame
    for (int i = 0; i < numOfFrames; i++)
    {
      printf("processing frame %d\n", i);
      frame = cv::imread(vstrImageFilenames[i], cv::IMREAD_UNCHANGED);

      // We now wrap the loaded image into a VPIImage object to be used by VPI.
      // VPI won't make a copy of it, so the original image must be in scope at all times.
      if (i == 0)
      {
        vpiImageCreateWrapperOpenCVMat(frame, 0, &vpiFrame);
      }
      else
      {
        vpiImageSetWrappedOpenCVMat(vpiFrame, frame);
      }

      // ---------------------
      // Process the frame
      // ---------------------

      // Convert to grayscale
      vpiCall(vpiSubmitConvertImageFormat, stream, backend, vpiFrame, vpiFrameGrayScale, nullptr);

      // Submit the pyramid generator
      vpiCall(vpiSubmitGaussianPyramidGenerator, stream, backend,
              vpiFrameGrayScale, pyrInput, VPI_BORDER_CLAMP);

      // Detect ORB features
      vpiCall(vpiSubmitORBFeatureDetector, stream, backend, orbPayload,
              pyrInput, keypoints, descriptors, &orbParams, VPI_BORDER_CLAMP);

      // TODO: Do we need this?
      vpiCall(vpiStreamSync, stream);

      // Stream it out
      int32_t numKeypoints;
      vpiCall(vpiArrayGetSize, keypoints, &numKeypoints);

      // Encode and Send
      orbStream.encodeAndSendFrame(keypoints, descriptors, numKeypoints, i);
    }

    vpiArrayDestroy(keypoints);
    vpiArrayDestroy(descriptors);
    vpiImageDestroy(vpiFrame);
    vpiImageDestroy(vpiFrameGrayScale);
    vpiPyramidDestroy(pyrInput);

    // Stop the timer
    timer.stop();
    printf("Processing time per frame: %f ms\n", timer.getTimeMilli() / numOfFrames);
  }
  catch (const VPIException &e)
  {
    std::cerr << e.what() << '\n';
    returnValue = -1;
  }

  // Cleanup
  vpiPayloadDestroy(orbPayload);
  vpiStreamDestroy(stream);

  return returnValue;
}

void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps)
{
  std::ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  while (!fTimes.eof())
  {
    std::string s;
    getline(fTimes, s);
    if (!s.empty())
    {
      std::stringstream ss;
      ss << s;
      vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e9);
    }
  }
}
