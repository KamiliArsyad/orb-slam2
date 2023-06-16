/**
 * Asynchronous Remote Computation Handler
 */
#include <opencv2/opencv.hpp>
#include <zmq.hpp>

#include <bitset>

using namespace std;

namespace ORB_SLAM2
{
  class ARCHandler
  {
  private:
    zmq::context_t context;
    zmq::socket_t socket;
    int port = 9999;
    std::string ip = "";

    struct FrameData
    {
      int frameNumber;
      int numKeypoints;
      std::bitset<256> *descriptors;
      cv::KeyPoint *keypoints;
    };

    std::bitset<256> decodeDescriptor(std::string encodedDescriptor);

    FrameData decodeFrameData(std::string encodedFrameData);

    FrameData receiveFrameData();

    bool isCorrectFrame(FrameData frameData);

    void writeKeypointsToVector(FrameData frameData, std::vector<cv::KeyPoint> &keypoints);

    void writeDescriptorsToOutputArray(FrameData frameData, cv::OutputArray descriptors);
  public:
    ARCHandler(std::string ip, int port, int queue_size);
    ~ARCHandler();

    void getFeatures(std::vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors);
  };
} // namespace ORB_SLAM2