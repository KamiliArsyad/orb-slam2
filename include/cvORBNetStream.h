#include <opencv2/core.hpp>
#include <bitset>
#include <string>
#include <zmq.hpp>

using namespace cv;

namespace ORB_SLAM2
{
  class cvORBNetStream
  {
  private:
    zmq::context_t context;
    zmq::socket_t socket;
    int port = 0;

    /**
     * Function to encode the keypoints and their respective descriptors of a frame
     * into a single string. The format is as follows:
     * <frameNumber>;<numKeypoints>;<desc1>;<x1>,<y1>;<desc2>;<x2>,<y2>;...
     * The descriptors are encoded as 32 characters ASCII strings for efficiency.
     */
    std::string encodeKeypoints(std::vector<KeyPoint> keypointsArray, Mat descriptorsArray, int numKeypoints, int frameNumber);

  public:
    cvORBNetStream(int port);
    ~cvORBNetStream();

    /**
     * Send an encoded frame.
     * @param encodedFrame The encoded frame to send.
     * @return 0 if successful, -1 otherwise.
     */
    int sendFrame(std::string encodedFrame);

    int encodeAndSendFrame(std::vector<KeyPoint> keypointsArray, Mat descriptorsArray, int numKeypoints, int frameNumber);
  };
} // namespace ORB_SLAM2
