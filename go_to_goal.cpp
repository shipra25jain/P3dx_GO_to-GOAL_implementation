#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
void callback(const nav_msgs::Odometry &msgs);
float error_pre;
int flag=1;int n=1;
 float error=0,error1=0,error2=0,ErroI=0,thetai=0,thetad=0,errort=0,thetad_p=0;

class SubscribeAndPublish
{      

    public:
    
      SubscribeAndPublish()
      {
   
         _pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);

   
       _sub=nh.subscribe("RosAria/pose",1000,&SubscribeAndPublish::callback,this);
      }

      void callback(const nav_msgs::Odometry &msgs)
        {

            double theta;
        float kit=.0001,kdt=0,kpt=.1;
        float ki=.10,kd=0.0,kp=1;

        int v=1;

        float yd=.50;
        tf::Quaternion q(msgs.pose.pose.orientation.x,msgs.pose.pose.orientation.y,msgs.pose.pose.orientation.z,msgs.pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        float xd=-.50;
        float yc,xc,I=0,D=0;
        int i=1;
        xc=msgs.pose.pose.position.x;
        yc=msgs.pose.pose.position.y;
        theta=atan2((yd-yc),(xd-xc));
        geometry_msgs::Twist msg;
        geometry_msgs::Pose pose_msg;

        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;

        _pub.publish(msg);
        error1 =yd-yc;
        error2 =xd-xc;
        error=sqrt(error1*error1+error2*error2);
        D=error-error_pre;
        I+=error;

        errort=(theta-yaw);
        thetai+=errort;
        thetad=errort-thetad_p;
        while(ros::ok && i)
        {
    if(!((errort<.01) && (errort>-.01))&& error>.1 )
                {
                                   ROS_INFO_STREAM("sendind"<<yaw<<"THETA"<<theta<<"    error"<<errort);

                    //msg.linear.x=0;                    
                //    if(errort>0)
                             {msg.angular.z=kpt*errort+kit*thetai+kdt*thetad;}
                //    else if(errort<0)
                //         {msg.angular.z=-(kpt*errort+kit*thetai+kdt*thetad);}          
                                msg.linear.x=0;
                msg.linear.y=0;
                                    
                    _pub.publish(msg);            
                }
    else if(/*(msgs.pose.pose.orientation.z<theta+.1)&&(msgs.pose.pose.orientation.z<theta+.05)*/(error >.01))
                {
                    
                    ROS_INFO_STREAM("hello"<<"  x"<<msgs.pose.pose.position.x<<"  y"<<msgs.pose.pose.position.y<<"error"<<error);
                     msg.linear.x= (kp*error + kd*D + ki*I)*sin(yaw);                                         //v*sin(yaw);
                     msg.linear.y =(kp*error + kd*D + ki*I)*cos(yaw);                      //v*cos(yaw);
                    msg.angular.z=0;
                    _pub.publish(msg);
                }
    else if(error<.01)
                {
                ROS_INFO_STREAM("hello");
                msg.angular.z=0;        
                msg.linear.x=0;                
                msg.linear.y=0;                    
            _pub.publish(msg);
            }
//        else {msg.angular.z=0;ROS_INFO_STREAM("Heloo");_pub.publish(msg);}

        i=i-1;error_pre=error;thetad_p=errort;
        }    

    }
    private:
    ros::NodeHandle nh;
      ros::Publisher _pub;
      ros::Subscriber _sub;
    
    

    };


int main(int argc, char **argv)
{

    //ErroI is used for integrating the error,error is for distance,error1 for x error

    ros::init(argc,argv,"go_to_goal");

    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}