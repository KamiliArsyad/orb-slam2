/**
 * This code is made for monocular eurocc dataset. No setting file is needed or used as of right now as we
 * are only ensure that the proposed model is working.
 */
#include <cvORBNetStream.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>

#include <cstdio>
#include <cstring> // for memset
#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>
#include <numeric>

#define USE_ANMS false

void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps);

/**
 * @brief Adaptive Non-Maximal Suppression (ANMS) using Supresion via Square Covering (SSC).
 *        The algorithm is based on the paper "Efficient adaptive non-maximal suppression algorithms
 *        for homogeneous spatial keypoint distribution" by Bailo, et al. (2018). The code is sourced
 *        from the repository referenced in the paper with some minor modifications (if any).
 * @param unsortedKeypoints The keypoints to be sorted and filtered.
 * @param numRetPoints The maximum number of keypoints to be returned.
 * @param tolerance The tolerance parameter for the SSC algorithm.
 * @param cols The number of columns in the image.
 * @param rows The number of rows in the image.
 */
std::vector<KeyPoint> ANMS_SSC(std::vector<KeyPoint> unsortedKeypoints, int numRetPoints, float tolerance, int cols, int rows);

/**
 * First argument: backend (<cpu|cuda>)
 * Second argument: the sequence directory
 */
int main(int argc, char *argv[])
{
  int returnValue = 0;

  // Parse parameters
  if (argc != 3)
  {
    throw std::runtime_error(std::string("Usage: ") + argv[0] + " <path_to_image_folder>" + " <path_to_times_file>");
  }

  std::vector<std::string> vstrImageFilenames;
  std::vector<double> vTimestamps;

  // Load images and timestamps
  LoadImages(argv[1], argv[2], vstrImageFilenames, vTimestamps);

  int numOfFrames = vstrImageFilenames.size();

  if (numOfFrames <= 0)
  {
    throw std::runtime_error("Couldn't load images");
  }

  //      CV ORB creation
  //      ---------------------
  std::vector<KeyPoint> keypoints;
  std::vector<KeyPoint> filteredKeypoints;
  Mat descriptors;
  int nFeatures = 10000;
  int limit = 500;
  Ptr<ORB> orb = ORB::create(USE_ANMS ? nFeatures : limit, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
  //      ---------------------

  // Initialize a timer
  cv::TickMeter timer;
  timer.start();

  cv::Mat frame;
  frame = cv::imread(vstrImageFilenames[0], cv::IMREAD_UNCHANGED);

  if (frame.empty())
  {
    throw std::runtime_error("Couldn't load image");
  }

  // Setup a worker stream
  ORB_SLAM2::cvORBNetStream orbStream(9999);

  // Process each frame
  for (int i = 0; i < numOfFrames; i++)
  {
    printf("processing frame %d\n", i);
    // This processs by itself takes around 40 ms to complete. Be wary of this as it is a huge bottleneck.
    frame = cv::imread(vstrImageFilenames[i], cv::IMREAD_UNCHANGED);

    // ---------------------
    // Process the frame
    // ---------------------
    // orb->detectAndCompute(frame, cv::Mat(), keypoints, descriptors);
    // int numKeyPoints = keypoints.size();
    orb->detect(frame, keypoints);
    if (USE_ANMS)
    {
      filteredKeypoints = ANMS_SSC(keypoints, limit, 0.1, frame.cols, frame.rows);
    }
    else
    {
      filteredKeypoints = keypoints;
    }
    orb->compute(frame, filteredKeypoints, descriptors);
    int numKeyPoints = filteredKeypoints.size();

    // Encode and Send
    orbStream.encodeAndSendFrame(filteredKeypoints, descriptors, numKeyPoints, i);
  }

  // Stop the timer
  timer.stop();
  printf("Processing time per frame: %f ms\n", timer.getTimeMilli() / numOfFrames);

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

std::vector<KeyPoint> ANMS_SSC(std::vector<KeyPoint> unsortedKeypoints, int numRetPoints, float tolerance, int cols, int rows)
{
  // Sort the keypoints by decreasing strength
  std::vector<float> responseVector;
  for (unsigned int i = 0; i < unsortedKeypoints.size(); i++)
    responseVector.push_back(unsortedKeypoints[i].response);
  std::vector<int> Indx(responseVector.size());
  std::iota(std::begin(Indx), std::end(Indx), 0);
  sortIdx(responseVector, Indx, cv::SORT_DESCENDING);

  // The sorted keypoints
  std::vector<KeyPoint> keypoints;
  for (unsigned int i = 0; i < unsortedKeypoints.size(); i++)
    keypoints.push_back(unsortedKeypoints[Indx[i]]);

  // several temp expression variables to simplify solution equation
  int exp1 = rows + cols + 2 * numRetPoints;
  long long exp2 =
      ((long long)4 * cols + (long long)4 * numRetPoints +
       (long long)4 * rows * numRetPoints + (long long)rows * rows +
       (long long)cols * cols - (long long)2 * rows * cols +
       (long long)4 * rows * cols * numRetPoints);
  double exp3 = sqrt(exp2);
  double exp4 = numRetPoints - 1;

  double sol1 = -round((exp1 + exp3) / exp4); // first solution
  double sol2 = -round((exp1 - exp3) / exp4); // second solution

  // binary search range initialization with positive solution
  int high = (sol1 > sol2) ? sol1 : sol2;
  int low = floor(sqrt((double)keypoints.size() / numRetPoints));
  low = max(1, low);

  int width;
  int prevWidth = -1;

  std::vector<int> ResultVec;
  bool complete = false;
  unsigned int K = numRetPoints;
  unsigned int Kmin = round(K - (K * tolerance));
  unsigned int Kmax = round(K + (K * tolerance));

  std::vector<int> result;
  result.reserve(keypoints.size());
  while (!complete)
  {
    width = low + (high - low) / 2;
    if (width == prevWidth ||
        low >
            high)
    {                     // needed to reassure the same radius is not repeated again
      ResultVec = result; // return the keypoints from the previous iteration
      break;
    }
    result.clear();
    double c = (double)width / 2.0; // initializing Grid
    int numCellCols = floor(cols / c);
    int numCellRows = floor(rows / c);
    std::vector<std::vector<bool>> coveredVec(numCellRows + 1,
                                              std::vector<bool>(numCellCols + 1, false));

    for (unsigned int i = 0; i < keypoints.size(); ++i)
    {
      int row =
          floor(keypoints[i].pt.y /
                c); // get position of the cell current point is located at
      int col = floor(keypoints[i].pt.x / c);
      if (coveredVec[row][col] == false)
      { // if the cell is not covered
        result.push_back(i);
        int rowMin = ((row - floor(width / c)) >= 0)
                         ? (row - floor(width / c))
                         : 0; // get range which current radius is covering
        int rowMax = ((row + floor(width / c)) <= numCellRows)
                         ? (row + floor(width / c))
                         : numCellRows;
        int colMin =
            ((col - floor(width / c)) >= 0) ? (col - floor(width / c)) : 0;
        int colMax = ((col + floor(width / c)) <= numCellCols)
                         ? (col + floor(width / c))
                         : numCellCols;
        for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov)
        {
          for (int colToCov = colMin; colToCov <= colMax; ++colToCov)
          {
            if (!coveredVec[rowToCov][colToCov])
              coveredVec[rowToCov][colToCov] =
                  true; // cover cells within the square bounding box with width
                        // w
          }
        }
      }
    }

    if (result.size() >= Kmin && result.size() <= Kmax)
    { // solution found
      ResultVec = result;
      complete = true;
    }
    else if (result.size() < Kmin)
      high = width - 1; // update binary search range
    else
      low = width + 1;
    prevWidth = width;
  }
  // retrieve final keypoints
  std::vector<KeyPoint> kp;
  for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keypoints[ResultVec[i]]);

  return kp;
}