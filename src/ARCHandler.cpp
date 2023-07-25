#include "ARCHandler.h"

namespace ORB_SLAM2
{
  /// @brief Construct a new ARCHandler::ARCHandler object
  /// @param ip the ip address of the server
  /// @param port the port of the server
  /// @param queue_size the size of the queue (not used yet)
  ARCHandler::ARCHandler(std::string ip, int port, int queue_size)
      : context(zmq::context_t(1)), socket(zmq::socket_t(context, zmq::socket_type::req))
  {
    this->ip = ip;
    this->port = port;
    std::string address = "tcp://" + ip + ":" + std::to_string(port);
    socket.connect(address);
  }

  ARCHandler::~ARCHandler()
  {
    socket.close();
  }

  std::bitset<256> ARCHandler::decodeDescriptor(std::string encodedDescriptor)
  {
    std::bitset<256> bits;
    for (std::size_t i = 0; i < encodedDescriptor.size(); ++i)
    {
      for (std::size_t j = 0; j < 8; ++j)
      {
        bits[i * 8 + j] = (encodedDescriptor[i] >> j) & 1;
      }
    }
    return bits;
  }

  ARCHandler::FrameData ARCHandler::decodeFrameData(std::string encodedKeypoints)
  {
    FrameData frameData;

    // Get the frame number
    std::string frameNumberStr = encodedKeypoints.substr(0, encodedKeypoints.find(";"));
    frameData.frameNumber = std::stoi(frameNumberStr);
    encodedKeypoints.erase(0, frameNumberStr.length() + 1);

    // Get the number of keypoints
    std::string numKeypointsStr = encodedKeypoints.substr(0, encodedKeypoints.find(";"));
    frameData.numKeypoints = std::stoi(numKeypointsStr);
    encodedKeypoints.erase(0, numKeypointsStr.length() + 1);

    // Allocate memory for the keypoints and descriptors
    frameData.keypoints = new cv::KeyPoint[frameData.numKeypoints];
    frameData.descriptors = new std::bitset<256>[frameData.numKeypoints];

    // Decode the keypoints and descriptors
    for (int i = 0; i < frameData.numKeypoints; ++i)
    {
      // Decode the descriptor
      std::string descriptorStr = encodedKeypoints.substr(0, 32);
      encodedKeypoints.erase(0, 33);
      std::bitset<256> descriptor = decodeDescriptor(descriptorStr);
      frameData.descriptors[i] = descriptor;

      // Decode the keypoint
      std::string keypointStr = encodedKeypoints.substr(0, encodedKeypoints.find(";"));
      encodedKeypoints.erase(0, keypointStr.length() + 1);
      std::string xStr = keypointStr.substr(0, keypointStr.find(","));
      std::string yStr = keypointStr.substr(keypointStr.find(",") + 1, keypointStr.length());

      cv::KeyPoint keypoint(std::stof(xStr), std::stof(yStr), 1);
      frameData.keypoints[i] = keypoint;
    }

    return frameData;
  }

  /// @brief Assert that the frame data is correct. Not yet implemented
  /// @param frameData the frame data to check
  /// @return true iff the frame data is correct
  bool ARCHandler::isCorrectFrame(FrameData frameData)
  {
    return true;
  }

  /// @brief Initiates the request to the connected device for the frame data.
  /// @return ARCHandler::FrameData the frame data
  ARCHandler::FrameData ARCHandler::receiveFrameData()
  {
    zmq::message_t reply;

    try
    {
      zmq::message_t request(1);
      memcpy(request.data(), "1", 1);

      // Make sure to block until the message is received
      socket.send(request, zmq::send_flags::none);
      socket.recv(reply, zmq::recv_flags::none);
    }
    catch (zmq::error_t e)
    {
      std::cout << "Error receiving frame: " << e.what() << std::endl;
      FrameData frameData;
      frameData.frameNumber = -1;
      return frameData;
    }

    std::string encodedFrame = std::string(static_cast<char *>(reply.data()), reply.size());
    FrameData frameData = decodeFrameData(encodedFrame);
    return frameData;
  }

  void ARCHandler::writeKeypointsToVector(FrameData frameData, std::vector<cv::KeyPoint> &keypoints)
  {
    for (int i = 0; i < frameData.numKeypoints; ++i)
    {
      keypoints.push_back(frameData.keypoints[i]);
    }
  }

  void ARCHandler::writeDescriptorsToOutputArray(FrameData frameData, cv::OutputArray descriptors)
  {
    cv::Mat descriptorsMat;

    // Initialize descriptors
    if (frameData.numKeypoints == 0)
    {
      descriptors.release();
    }
    else
    {
      descriptors.create(frameData.numKeypoints, 32, CV_8U);
      descriptorsMat = descriptors.getMat();
    }

    for (int i = 0; i < frameData.numKeypoints; ++i)
    {
      unsigned char *ptr = descriptorsMat.ptr<unsigned char>(i);
      std::bitset<256> &desc = frameData.descriptors[i];

      for (int i = 0; i < 32; ++i)
      {
        unsigned char byte = 0;

        for (int j = 0; j < 8; ++j)
        {
          byte |= desc[i * 8 + j] << j;
        }

        ptr[i] = byte;
      }
    }
  }

  void ARCHandler::getFeatures(vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors)
  {
    FrameData frameData = receiveFrameData();
    if (!isCorrectFrame(frameData))
    {
      return;
    }

    writeKeypointsToVector(frameData, keypoints);
    writeDescriptorsToOutputArray(frameData, descriptors);
  }

  void ARCHandler::getFeaturesFast(cv::Mat &descriptors, std::vector<cv::KeyPoint> &keypoints, cv::Mat &image)
  {
    zmq::message_t message;

    try
    {
      zmq::message_t request(1);
      memcpy(request.data(), "1", 1);

      // Make sure to block until the message is received
      socket.send(request, zmq::send_flags::none);
      socket.recv(message, zmq::recv_flags::none);

      // Get descriptors
      // Get the frame number
      int frameNumber;
      memcpy(&frameNumber, message.data(), 4);

      // Get the number of keypoints
      unsigned short numKeypoints;
      memcpy(&numKeypoints, static_cast<char *>(message.data()) + 4, 2);

      // Get the descriptors
      descriptors = cv::Mat(numKeypoints, 32, CV_8UC1);
      memcpy(descriptors.data, static_cast<char *>(message.data()) + 6, 32 * numKeypoints);

      // Get the keypoints
      keypoints.clear();
      for (int i = 0; i < numKeypoints; ++i)
      {
        unsigned short x, y;
        memcpy(&x, static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * i, 2);
        memcpy(&y, static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * i + 2, 2);

        keypoints.push_back(cv::KeyPoint(x, y, 1));
      }

      // Get the image
      uint32_t imgSize;
      memcpy(&imgSize, static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints, 4);

      std::vector<uchar> data(static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints + 4,
                              static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints + 4 +
                                  imgSize);

      image = cv::imdecode(data, cv::IMREAD_COLOR); 
    }
    catch (zmq::error_t e)
    {
      std::cout << "Error receiving frame: " << e.what() << std::endl;
    }
  }
}