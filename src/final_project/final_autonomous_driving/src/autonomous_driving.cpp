#include <math.h>
#include <ros/ros.h>
#include <string>
#include "autonomous_msg/PolyfitLaneDataArray.h"
#include "autonomous_msg/VehicleInput.h"
#include "autonomous_msg/VehicleOutput.h"
#include "std_msgs/Float64.h"

class AutonomousDriving {
 protected:
  ros::NodeHandle m_rosNodeHandler;
  ros::Publisher m_rosPubVehicleInput;

  ros::Subscriber m_rosSubVehicleState;
  ros::Subscriber m_rosSubPolyLanes;
  ros::Subscriber m_rosSubDrivingInput;
  ros::Subscriber m_rosLimitSpeed;
  ros::Subscriber m_rosSubLidar;

  ros::Publisher m_rosPubDrivingWay;

  std::string m_vehicle_namespace_param;
  double m_lookAhead_param = 0.0;
  
 public:
  AutonomousDriving() {
    m_rosSubVehicleState = m_rosNodeHandler.subscribe(
        "vehicle_output", 10, &AutonomousDriving::vehicleStateCallback, this);

    m_rosPubVehicleInput =
        m_rosNodeHandler.advertise<autonomous_msg::VehicleInput>(
            "vehicle_input", 10);

    m_rosSubPolyLanes = m_rosNodeHandler.subscribe(
        "polyfit_lanes", 10, &AutonomousDriving::polyLanesCallback, this);

    m_rosLimitSpeed = m_rosNodeHandler.subscribe(
        "/limit_speed", 10, &AutonomousDriving::limitSpeedCallback, this);

    m_rosSubDrivingInput = m_rosNodeHandler.subscribe(
        "/driving_input", 10, &AutonomousDriving::drivingInputCallback, this);

    m_rosSubLidar = m_rosNodeHandler.subscribe(
        "/lidar", 10, &AutonomousDriving::lidarSensorCallback, this);

    m_rosPubDrivingWay =
        m_rosNodeHandler.advertise<autonomous_msg::PolyfitLaneData>(
            "driving_way", 10);

    m_rosNodeHandler.param("autonomous_driving/ns", m_vehicle_namespace_param,
                           std::string(""));
    m_rosNodeHandler.param("autonomous_driving/lookAhead", m_lookAhead_param,
                           5.0);
  }

  ~AutonomousDriving() {}

 protected:
  autonomous_msg::PolyfitLaneDataArray m_polyLanes;
  autonomous_msg::VehicleInput m_drivingInput;
  autonomous_msg::PolyfitLaneData m_midPolyLane;
  autonomous_msg::VehicleOutput m_vehicleState;
  autonomous_msg::VehicleOutput m_lidarOutput;
  double m_limitSpeed = 0.0;
  bool isLidarDataExist = false;
  
  float distance = 0.0;
  

 public:
  /**
   * @brief Temporary functions for debugging pure pursuit
   *
   * @param VehicleInput accel and brake
   */
  void drivingInputCallback(const autonomous_msg::VehicleInput::ConstPtr &msg) {
    m_drivingInput.accel = msg->accel;
    m_drivingInput.brake = msg->brake;
    // m_drivingInput.steering = msg->steering;
  }

  void vehicleStateCallback(
      const autonomous_msg::VehicleOutput::ConstPtr &msg) {
    m_vehicleState = *msg;
  }

  void limitSpeedCallback(const std_msgs::Float64::ConstPtr &msg) {
    m_limitSpeed = msg->data;

  }

  void polyLanesCallback(
    const autonomous_msg::PolyfitLaneDataArray::ConstPtr &msg) {
    m_polyLanes = *msg;
  }
  void lidarSensorCallback(const autonomous_msg::VehicleOutput::ConstPtr &msg)
  {
    m_lidarOutput = *msg;
    isLidarDataExist = true;
    distance = sqrt((m_lidarOutput.x - m_vehicleState.x)*(m_lidarOutput.x - m_vehicleState.x) + (m_lidarOutput.y - m_vehicleState.y)*(m_lidarOutput.y - m_vehicleState.y));
    
  }

  void controlVehicleSpeed(double targetSpeed_ms) {
    // TODO
     
 
    
       if ( m_lidarOutput.velocity > 0.1 && distance < 10 + m_lidarOutput.velocity)
   {
      
      m_limitSpeed = m_lidarOutput.velocity;
      
   
   }
   else
   {
      m_limitSpeed = 20.0;

      
   }
   

    if (m_vehicleState.velocity >= m_limitSpeed)
    {
      
      m_drivingInput.brake = 1;
      if (distance <= 10 && distance > 0)
      {
          m_drivingInput.brake = 10;
      }
      
       
    }
    else
    {
      
       m_drivingInput.accel = targetSpeed_ms;
      
      if (distance <= 10 && distance > 0)
      {
          m_drivingInput.brake = 10;
      }
      else if ( distance >= 10 && distance <= 10 + m_lidarOutput.velocity)
      {
          
      }
      
    }   
   
       
  }
       
  

  void findDrivingWay() 
  {
   auto lanenumber = m_polyLanes.polyfitLanes.size();
   float_t max[4];
   float_t min[4];
   float_t lane1[4];
   float_t lane2[4];
   
   if (lanenumber ==0)
   {
      m_midPolyLane.a0 = m_midPolyLane.a0;
        m_midPolyLane.a1 = m_midPolyLane.a1;
        m_midPolyLane.a2 = m_midPolyLane.a2;
        m_midPolyLane.a3 = m_midPolyLane.a3;
   } 
   else if (lanenumber == 2)
   {
     if (  (abs(m_polyLanes.polyfitLanes[0].a0) + abs(m_polyLanes.polyfitLanes[1].a0 ))/2 <= 1.90 )
       {      
         lane1[0]=m_polyLanes.polyfitLanes[0].a0;
         lane1[1]=m_polyLanes.polyfitLanes[0].a1;
         lane1[2]=m_polyLanes.polyfitLanes[0].a2;
           lane1[3]=m_polyLanes.polyfitLanes[0].a3;

         lane2[0]=m_polyLanes.polyfitLanes[1].a0;
              lane2[1]=m_polyLanes.polyfitLanes[1].a1;
              lane2[2]=m_polyLanes.polyfitLanes[1].a2;
         lane2[3]=m_polyLanes.polyfitLanes[1].a3;

              m_midPolyLane.a0 = (lane1[0] +lane2[0])/2;
              m_midPolyLane.a1 = (lane1[1] +lane2[1])/2;
              m_midPolyLane.a2 = (lane1[2] +lane2[2])/2;
              m_midPolyLane.a3 = (lane1[3] +lane2[3])/2; 
         }
   }
   else if (lanenumber > 2)
   {   
   
   for(int i = 0; i<lanenumber; i++) 
        {
           if ( m_polyLanes.polyfitLanes[i].a0 > 3.5 )
           {
       max[0] = m_polyLanes.polyfitLanes[i].a0;
       max[1] = m_polyLanes.polyfitLanes[i].a1;
       max[2] = m_polyLanes.polyfitLanes[i].a2;
       max[3] = m_polyLanes.polyfitLanes[i].a3;
           }
           else if( m_polyLanes.polyfitLanes[i].a0 < -3.5 )
           {
              
       min[0] = m_polyLanes.polyfitLanes[i].a0;
       min[1] = m_polyLanes.polyfitLanes[i].a1;
       min[2] = m_polyLanes.polyfitLanes[i].a2;
       min[3] = m_polyLanes.polyfitLanes[i].a3;
           }
        }
   

   
   

         m_midPolyLane.a0 = (min[0] + max[0])/2;
        m_midPolyLane.a1 = (min[1] + max[1])/2;
        m_midPolyLane.a2 = (min[2] + max[2])/2;
        m_midPolyLane.a3 = (min[3] + max[3])/2;


   
   }
    
    m_midPolyLane.frame_id = m_vehicle_namespace_param + "/body";
    m_rosPubDrivingWay.publish(m_midPolyLane);
}



  /**
   * brief: Find the steering angle for driving in the driving lane.
   * input: m_midPolyLane
   * output: m_drivingInput.steering
   */

void calcSteeringAngle() {

      double gx = 8;
      double gy = m_midPolyLane.a0 + m_midPolyLane.a1*gx + m_midPolyLane.a2*gx*gx + m_midPolyLane.a3*gx*gx*gx;

      double L = 1.402 + 1.646;

      double ld = sqrt(gx*gx + gy*gy);
      double eld = gy;

      if(ld==0) {
      ld = INT_MIN;}
         

      m_drivingInput.steering = atan(2*L*eld/(ld*ld));

    }


  void publishVehicleInput() { m_rosPubVehicleInput.publish(m_drivingInput); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "autonomous_driving");

  AutonomousDriving autonomousDriving;

  double prev_csvLaneMarkTime = ros::Time::now().toSec();
  // The approximate control time is 100 Hz
  ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok()) {

   
    
    autonomousDriving.controlVehicleSpeed(1);
    
    autonomousDriving.findDrivingWay();
    autonomousDriving.calcSteeringAngle();
    autonomousDriving.publishVehicleInput();

    if ((ros::Time::now().toSec() - prev_csvLaneMarkTime) > 1.0) {
      prev_csvLaneMarkTime = ros::Time::now().toSec();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
