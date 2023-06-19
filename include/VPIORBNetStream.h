#include <vpi/Array.h>
#include <vpi/Stream.h>

#include <bitset>
#include <string>

#include <zmq.hpp>

namespace ORB_SLAM2
{
  class VPIORBNetStream
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
    std::string encodeKeypoints(VPIArray keypointsArray, VPIArray descriptorsArray, int numKeypoints, int frameNumber);

  public:
    VPIORBNetStream(int port);
    ~VPIORBNetStream();

    /**
     * Send an encoded frame.
     * @param encodedFrame The encoded frame to send.
     * @return 0 if successful, -1 otherwise.
     */
    int sendFrame(std::string encodedFrame);

    int encodeAndSendFrame(VPIArray keypointsArray, VPIArray descriptorsArray, int numKeypoints, int frameNumber);
  };
} // namespace ORB_SLAM2
