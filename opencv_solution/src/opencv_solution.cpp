#include "ros/ros.h"
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "vision_msgs/Detection2DArray.h"

// Declare namespaces to clean up code
using namespace cv;
using namespace cv_bridge;
using namespace std;

// Create instance of publishers
image_transport::Publisher imagePublisher;
ros::Publisher detectionPublisher;

// Create instance of hog descriptor
HOGDescriptor hog;

// Callback for image received
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Declare vector to store found bounding boxes
	vector<Rect> found;

    // Create output message and copy header
	vision_msgs::Detection2DArray detectionMsg;
	detectionMsg.header = msg->header;

    // Declare pointer to image
	CvImagePtr cv_ptr;

    // Try to convert image to opencv type. If it fails, raise error
    try
    {
		cv_ptr = toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
    catch (cv_bridge::Exception& e)
    {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	return;
    }
	
    // Store converted image
    Mat mat_image = cv_ptr->image;
    
    // Detect persons and store the resulting bounding boxes in the found vector
    hog.detectMultiScale(mat_image,found,0,Size(8,8));

    // Loop over found vector to create messages in the type of Detection2D
    // and add them to the output message
    for (int i = 0; i<found.size();i++)
    {
        Rect r = found[i];
        rectangle(mat_image,r.tl(),r.br(),Scalar(0,255,0),2);
        vision_msgs::Detection2D detection;
        detection.bbox.size_x = r.width;
        detection.bbox.size_y = r.height;
        detection.bbox.center.x = r.x + r.width/2;
        detection.bbox.center.y = r.y - r.height/2;
        detectionMsg.detections.push_back(detection);

    }

    // Create a new output message for the visual topic
    sensor_msgs::ImagePtr outMsgVisual = CvImage(std_msgs::Header(), "bgr8",mat_image).toImageMsg();

    // Publish on both topics
    imagePublisher.publish(outMsgVisual);
    detectionPublisher.publish(detectionMsg);

}

int main(int argc, char **argv)
{
    // Initialize ros
    ros::init(argc, argv, "opencv_solution_node");

    // Create instance of the nodehandler
    ros::NodeHandle nh;

    // Create instance of the ImageTransport, which provides a framework to send and receive images
    image_transport::ImageTransport it(nh);

    // Subscribe to the raw images using the ImageTransport framework 
    image_transport::Subscriber sub = it.subscribe("/prius/front_camera/image_raw",1,imageCallback);

    // Add publisher to the ImageTransport framework
    imagePublisher = it.advertise("/opencv_solution_node/visual",1000);

    // Add publisher to the nodehandler
    detectionPublisher = nh.advertise<vision_msgs::Detection2DArray>("/opencv_solution_node/detections",1000);

    // Configure hog detector using default people detector
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    
    // let it spin!
    ros::spin();
}
