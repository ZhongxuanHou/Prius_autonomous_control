#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection3DArray.h"
#include "prius_msgs/Control.h"

class Controller
{
  private:
    ros::NodeHandle _nh;
    std::vector<vision_msgs::Detection2D> _openCVDetections;
    std::vector<vision_msgs::Detection3D> _PCLDetections;
    ros::Publisher _controlPublisher;
    ros::Subscriber _openCVSubscriber;
    ros::Subscriber _PCLSubscriber;
    bool _pedestrianSpotted;

  public:
    Controller(ros::NodeHandle nh)
    {
        _nh = nh;
        _openCVSubscriber = _nh.subscribe("/opencv_solution_node/detections", 1, &Controller::openCVCallback,this);
        _PCLSubscriber = _nh.subscribe("/pcl_solution_node/detections", 1, &Controller::PCLCallback,this);
        _controlPublisher = _nh.advertise<prius_msgs::Control>("/prius", 1000);
        _pedestrianSpotted = false;
    };

    // Calculate the distance between the object and the vehicle
    float calculateRadius(vision_msgs::Detection3D object)
    {
        float a = object.bbox.center.position.x;
        float b = object.bbox.center.position.y;
        float r = sqrt(a*a + b*b);
        return r;
    };

    void controlVehicle(void)
    {
        // Create output message instance
        prius_msgs::Control outMessage;

        // Check if a pedestrian is spotted
        if (!_openCVDetections.empty() && !_pedestrianSpotted)
        {
            _pedestrianSpotted = true;
        }

        // If a pedestrian is spotted, brake the car and exit this function
        if (_pedestrianSpotted)
        {
            outMessage.throttle = 0;
            outMessage.steer = 0;
            outMessage.brake = 1;
            _controlPublisher.publish(outMessage);
            return;
        }

        // Check if a pylon has been detected
        if (!_PCLDetections.empty())
        {
            // Set first detected pylon as closest and calculate the distance from the car to the pylon
            vision_msgs::Detection3D closestObject = _PCLDetections[0];
            float closestRadius = calculateRadius(closestObject);

            // Itterate over found pylons to find the closest
            for (int i = 1; i < _PCLDetections.size(); i++)
            {
                if (calculateRadius(_PCLDetections[i]) < closestRadius)
                {
                    closestObject = _PCLDetections[i];
                    closestRadius = calculateRadius(closestObject);
                }
            }

            // Steer away from the pylon
            if (closestObject.bbox.center.position.y > 0)
            {
                outMessage.steer = -1;
            }
            else
            {
                outMessage.steer = 1;
            }
        }
        // If no pylons are detected, drive straight
        else
        {
            outMessage.steer = 0;
        }

        // Set throttle
        outMessage.throttle = 1;

        // Send message
        _controlPublisher.publish(outMessage);
    }

    void openCVCallback(const vision_msgs::Detection2DArray &msg)
    {
        // Clear any previously found detections
        _openCVDetections.clear();

        // Itterate over the detections in the message
        for (int i = 0; i < msg.detections.size(); i++)
        {
            // If the bounding box is larger than 25000 pixels in area, add it to the vector
            if (msg.detections[i].bbox.size_x * msg.detections[i].bbox.size_y >= 25000)
                _openCVDetections.push_back(msg.detections[i]);
        }

        // Update the controls on the vehicle
        controlVehicle();
    };

    void PCLCallback(const vision_msgs::Detection3DArray &msg)
    {
        // Clear any previously found detections
        _PCLDetections.clear();

        // Itterate over the detections in the message
        for (int i = 0; i < msg.detections.size(); i++)
        {
            // If a detection is in front of the vehicle
            if (msg.detections[i].bbox.center.position.x > 0)
            {
                // If a detection is within 6 meters of the vehicle, add it to the vector
                if (calculateRadius(msg.detections[i]) < 6)
                    _PCLDetections.push_back(msg.detections[i]);
            }
        }

        // Update the controls on the vehicle
        controlVehicle();
    };
};

#endif
