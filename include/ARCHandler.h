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

  public:
    ARCHandler(std::string ip, int port, int queue_size);
    ~ARCHandler();

    void getFeaturesFast(cv::Mat &descriptors, std::vector<cv::KeyPoint> &keypoints, cv::Mat &image, double &timeStamp);
  };
} // namespace ORB_SLAM2