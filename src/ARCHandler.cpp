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

  void ARCHandler::getFeaturesFast(cv::Mat &descriptors, std::vector<cv::KeyPoint> &keypoints, cv::Mat &image, double &timeStamp)
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

      // Get the timestamp (double)
      memcpy(&timeStamp, static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints, 8);

      // Get the image if it exists
      if (message.size() <= 6 + 32 * numKeypoints + 4 * numKeypoints + 8)
      {
        return;
      }

      uint32_t imgSize;
      memcpy(&imgSize, static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints + 8, 4);

      std::vector<uchar> data(static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints + 4 + 8,
                              static_cast<char *>(message.data()) + 6 + 32 * numKeypoints + 4 * numKeypoints + 4 + 8 +
                                  imgSize);

      image = cv::imdecode(data, cv::IMREAD_COLOR);
    }
    catch (zmq::error_t e)
    {
      std::cout << "Error receiving frame: " << e.what() << std::endl;
    }
  }
}