#include "VPIORBNetStream.h"
#include <iostream>

// If true the program will block the thread and wait for message to be received before sending the next frame
#define BLOCKING_MODE true

namespace ORB_SLAM2
{
  // Constructor and destructor for the VPIORBNetStream class
  VPIORBNetStream::VPIORBNetStream(int port)
      : port(port), context(zmq::context_t(1)), socket(zmq::socket_t(context, zmq::socket_type::rep))
  {
    socket.bind("tcp://*:" + std::to_string(port));
  }

  VPIORBNetStream::~VPIORBNetStream()
  {
    socket.close();
  }

  /**
   * Function to encode the keypoints and their respective descriptors of a frame
   * into a single string. The format is as follows:
   * <frameNumber>;<numKeypoints>;<desc1>;<x1>,<y1>;<desc2>;<x2>,<y2>;...
   * The descriptors are encoded as 32 characters ASCII strings for efficiency.
   */
  std::string VPIORBNetStream::encodeKeypoints(VPIArray keypointsArray, VPIArray descriptorsArray, int numKeypoints, int frameNumber)
  {
    std::ostringstream ss;
    ss << frameNumber << ";" << numKeypoints << ";";

    // Lock the arrays to access their data
    VPIArrayData keypointsData, descriptorsData;
    vpiArrayLockData(keypointsArray, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &keypointsData);
    vpiArrayLockData(descriptorsArray, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &descriptorsData);
    vpiArrayUnlock(keypointsArray);
    vpiArrayUnlock(descriptorsArray);

    VPIKeypointF32 *keypoints = (VPIKeypointF32 *)keypointsData.buffer.aos.data;

    // Encode the keypoints and descriptors
    for (int i = 0; i < numKeypoints; ++i)
    {
      // Copy the descriptor to a bitset
      std::bitset<256> descriptor; 

      for (int j = 0; j < 4; ++j)
      {
        descriptor |= ((uint64_t *)descriptorsData.buffer.aos.data)[i * 4 + j];
        if (j < 3) descriptor <<= 64;
      }
     
      // Encode the descriptor to 32 characters
      for (int j = 0; j < 32; ++j)
      {
        unsigned char byte = 0;

        for (int k = 0; k < 8; ++k)
        {
          byte |= descriptor[j * 8 + k] << k;
        }

        ss << byte; 
      }
      
      ss << ";";

      // Encode the keypoint
      ss << keypoints[i].x << "," << keypoints[i].y << ";";
    }

    return ss.str();
  }

  int VPIORBNetStream::sendFrame(std::string encodedFrame)
  {
    zmq::message_t request(encodedFrame.size());
    memcpy(request.data(), encodedFrame.c_str(), encodedFrame.size());

    // TODO: What if there's no request? Add a buffer with a safe-distancing semaphore.
    try
    {
      zmq::message_t temp;
      socket.recv(temp, BLOCKING_MODE ? zmq::recv_flags::none : zmq::recv_flags::dontwait);
      socket.send(request, BLOCKING_MODE ? zmq::send_flags::none : zmq::send_flags::dontwait); 
    }
    catch (zmq::error_t e)
    {
      std::cout << "Error sending frame: " << e.what() << std::endl;
      return -1;
    }

    return 0;
  }

  /// @brief Encode and send a processed frame feature descriptors and keypoints.
  /// @param keypointsArray
  /// @param descriptorsArray 
  /// @param numKeypoints 
  /// @param frameNumber 
  /// @return 0 if successful, -1 otherwise.
  int VPIORBNetStream::encodeAndSendFrame(VPIArray keypointsArray, VPIArray descriptorsArray, int numKeypoints, int frameNumber)
  {
    std::string encodedFrame = encodeKeypoints(keypointsArray, descriptorsArray, numKeypoints, frameNumber);
    return sendFrame(encodedFrame);
  }
} // namespace ORB_SLAM2