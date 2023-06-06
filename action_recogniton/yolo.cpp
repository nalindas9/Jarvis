/*
YOLOv3 and Real Time Openpose Hand Keypoints and Object Detection

Reference: 
1. https://github.com/spmallick/learnopencv/blob/master/ObjectDetection-YOLO/object_detection_yolo.cpp
2. https://github.com/spmallick/learnopencv/blob/master/HandPose/handPoseImage.cpp
3. https://github.com/opencv/opencv/blob/master/samples/dnn/openpose.cpp

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing M.Eng. in Robotics,
University of Maryland, College Park
*/

#include <fstream>
#include <sstream>
#include <iostream>
#include <tuple>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

using namespace cv;
using namespace dnn;
using namespace std;

const int POSE_PAIRS[4][24][2] = {
{   // COCO body
    {1,2}, {1,5}, {2,3},
    {3,4}, {5,6}, {6,7},
    {1,8}, {8,9}, {9,10},
    {1,11}, {11,12}, {12,13},
    {1,0}, {0,14},
    {14,16}, {0,15}, {15,17}
},
{   // MPI body
    {0,1}, {1,2}, {2,3},
    {3,4}, {1,5}, {5,6},
    {6,7}, {1,14}, {14,8}, {8,9},
    {9,10}, {14,11}, {11,12}, {12,13}
},
{   // hand
    {0,1}, {1,2}, {2,3}, {3,4},         // thumb
    {0,5}, {5,6}, {6,7}, {7,8},         // pinkie
    {0,9}, {9,10}, {10,11}, {11,12},    // middle
    {0,13}, {13,14}, {14,15}, {15,16},  // ring
    {0,17}, {17,18}, {18,19}, {19,20}   // small
},
{  // Body_25 body and foot
    {1, 0}, {1, 2}, {1, 5},
    {2, 3}, {3, 4}, {5, 6},
    {6, 7}, {0, 15}, {15, 17},
    {0, 16}, {16, 18}, {1, 8},
    {8, 9}, {9, 10}, {10, 11},
    {11, 22}, {22, 23}, {11, 24},
    {8, 12}, {12, 13}, {13, 14}, 
    {14, 19}, {19, 20}, {14, 21}
}
};


// Initialize the parameters
float confThreshold = 0.5; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
vector<string> classes;

// Remove the bounding boxes with low confidence using non-maxima suppression
vector<tuple<Point, string>> postprocess(Mat& frame, const vector<Mat>& out);

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net);

// Action Premitives
vector<string> actions = {"picked", "placed"};
// Draw the predicted bounding box
tuple<Point, string> drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);

// Find euclidean distance between two points
tuple<float, string> nearest_obj(Point &p1, vector<tuple<Point, string>> o1);
float mod = 0, old_mod = 0, yy = 0, old_yy = 0;
int nPoints = 26;
int midx = 3; int nPairs = 24; int nparts = 25;
bool placed = true;
string object_picked;
tuple<float, string> object_nearest; 
// Main Function
int main(int argc, char** argv){

  float thresh = 0.1;
  // Configuration file name and  Weight file name
  string modelConfiguration = "/home/nalindas9/openpose/cfg/yolov3.cfg";
  string modelWeights = "/home/nalindas9/openpose/weights/yolov3.weights";
  string protoFile = "/home/nalindas9/openpose/models/pose/body_25/pose_deploy.prototxt";
  string weightsFile = "/home/nalindas9/openpose/models/pose/body_25/pose_iter_584000.caffemodel";
  //string imageFile = "hand.jpg";
  vector<tuple<Point, string>>  object_pts;
  
  Point wrist;
  size_t pos;
  // Load names of classes
  string classesFile = "/home/nalindas9/openpose/data/coco.names";
  ifstream ifs(classesFile.c_str());
  string line;
  while (getline(ifs, line)) classes.push_back(line);
      
  // Load the network
  Net net = readNetFromDarknet(modelConfiguration, modelWeights);
  Net bodynet = readNetFromCaffe(protoFile, weightsFile);
  net.setPreferableBackend(DNN_BACKEND_CUDA);
  net.setPreferableTarget(DNN_TARGET_CUDA);
  bodynet.setPreferableBackend(DNN_BACKEND_CUDA);
  bodynet.setPreferableTarget(DNN_TARGET_CUDA);
  // Open Webcam
  VideoCapture cam(0);

  if(!cam.isOpened()){
    cout << "Could not open camera :( Exiting ...\n";
    return -1;
  }

  Mat frame, blob;
  cout << "Loaded Network! Ready for Detection ..." << endl;
  cout << "Press the ESC key to end detection.\n";
  // Operate on each frame
  while(1){
    //auto start = chrono::steady_clock::now();
    cam >> frame;
    // Create a 4d Blob from the frame
    blobFromImage(frame, blob, 1/255.0, Size(416, 416), Scalar(0,0,0), false, false);

    // Set the i/p to the network
    net.setInput(blob);
    bodynet.setInput(blob);

    // Runs the forward pass to get the output of the network
    vector<Mat> outs;
    net.forward(outs, getOutputsNames(net));
    Mat output = bodynet.forward();
    int H = output.size[2];
    int W = output.size[3];

    // find the position of the body parts
    vector<Point> points(nPoints);

    // Remove bounding box with low confidence
    object_pts = postprocess(frame, outs);
    //pos = (get<1>(object_pts)).find(":");
    
    // Write the frame with the detection boxes
    //Mat detectedFrame;
    //frame.convertTo(detectedFrame, CV_8U);
    

    for (int n=0; n < nparts; n++)
    {
        old_mod = mod;
        old_yy = yy;
        // Probability map of corresponding body's part.
        Mat probMap(H, W, CV_32F, output.ptr(0,n));

        Point2f p(-1,-1);
        Point maxLoc;
        double prob;
        minMaxLoc(probMap, 0, &prob, 0, &maxLoc);
        if (prob > thresh)
        {
            p = maxLoc;
            p.x *= (float)frame.cols / W ;
            p.y *= (float)frame.rows / H ;

            circle(frame, cv::Point((int)p.x, (int)p.y), 8, Scalar(0,255,255), -1);
            cv::putText(frame, cv::format("%d", n), cv::Point((int)p.x, (int)p.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            if(n == 4){
              // Getting the top left coords of the bottle
              wrist = cv::Point((int)p.x, (int)p.y);
              mod = sqrtf(wrist.x*wrist.x + wrist.y*wrist.y);
              yy = wrist.y;
              object_nearest = nearest_obj(wrist, object_pts);
              //cout << "Nearest object is " << get<1>(object_nearest) << " with distance " << get<0>(object_nearest) << " and change " << yy - old_yy << endl;
              //cout << dist(wrist, get<0>(object_pts)) << endl;
              if(get<0>(object_nearest) <= 90){
                object_picked = get<1>(object_nearest);
                if(((yy - old_yy) <= -10) && placed){
                  cout << "Human" << " " << actions[0] << " " << object_picked << endl;
                  placed = false;
                  }
                }else if(get<0>(object_nearest) > 40 && ((yy - old_yy) <= -10) && !placed && get<1>(object_nearest) == object_picked){
                  placed = true;
                  cout << "Human" << " " << actions[1] << " " << object_picked << endl;
                }else
                  continue;
              
                //if(abs(mod - old_mod) >= 20)   
                  //cout << "Human" << " " << actions[0] << " " << (get<1>(object_pts)).substr(0, pos) << endl;
              //if ((get<1>(object_pts)).substr(0, pos) != "person")
                //cout << "Distance between " << (get<1>(object_pts)).substr(0, pos) << " and wrist is " << dist(wrist, get<0>(object_pts)) << endl;
            }
        }
        points[n] = p;
    }
    
    
    
    //int nPairs = sizeof(POSE_PAIRS)/sizeof(POSE_PAIRS[0]);
    for (int n = 0; n < nPairs; n++)
    {
        // lookup 2 connected body/hand parts
        Point2f partA = points[POSE_PAIRS[midx][n][0]];
        Point2f partB = points[POSE_PAIRS[midx][n][1]];
        
        if (partA.x<=0 || partA.y<=0 || partB.x<=0 || partB.y<=0)
            continue;
        
        cv::line(frame, partA, partB, Scalar(0,255,255), 8);
        circle(frame, partA, 8, Scalar(0,0,255), -1);
        circle(frame, partB, 8, Scalar(0,0,255), -1);
    }
    imshow("Detected Frame", frame);
    
    //auto end = chrono::steady_clock::now();
    //cout << "Elapsed time in milliseconds : "+
	//	<< chrono::duration_cast<chrono::milliseconds>(end - start).count()
	//	<< " ms" << endl;
    char k = waitKey(1);
    if(k == 27)
      break;
  }

  // Release the Video capture object and destroy all windows
  cam.release();
  destroyAllWindows();

  cout << "Execution ended!\n";

  return 0;
}

// Remove the bounding boxes with low confidence using non-maxima suppression
vector<tuple<Point, string>>  postprocess(Mat& frame, const vector<Mat>& outs)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    vector<tuple<Point, string>> objects;
    tuple<Point, string> object;
    size_t pos;
    
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }
    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        object = drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame);
        pos = (get<1>(object)).find(":");
        if((get<1>(object)).substr(0, pos) != "person")  
          objects.push_back(object);
    }
    return objects;
}

// Draw the predicted bounding box
tuple<Point, string> drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 3);
    
    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    
    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0),1);
    
    return make_tuple(Point(left, top), label);
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();
        
        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
        names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
// Find euclidean distance between two points
tuple<float, string> nearest_obj(Point &p1, vector<tuple<Point, string>> o1){
  Point p2;
  float least_dist = 1000;
  float dist;
  string nearest_object;
  size_t pos;
  for(int i = 0; i < o1.size(); ++i){
    p2 = get<0>(o1[i]);
    dist = sqrtf((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
    if(dist < least_dist){
      pos = (get<1>(o1[i])).find(":");
      nearest_object = (get<1>(o1[i])).substr(0, pos);
      least_dist = dist;  
     }  
  }
  return make_tuple(least_dist , nearest_object);
}
