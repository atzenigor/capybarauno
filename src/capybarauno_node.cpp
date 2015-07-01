//system stuff
#include <string>
//ros stuff
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "capybarauno/capybara_ticks.h"             //no needed :To Delete
#include "capybarauno/capybara_ticks_signed.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

//own stuff
#include "malComm/mal_comm.h"
#include "malComm/mal_primitives.h"

using namespace std;

// Serial port variables
int serialFd;
struct Packet_Decoder packet_decoder;
struct Packet packet;
struct Speed_Payload speedPayload;
struct State_Payload statePayload;

// heartbeat variables
int beatcnt=0;
int beatingheart=0;
struct Packet heartbeat_packet;
struct Heartbeat_Payload heartbeat;

// courrent relative ticks
int16_t leftSignedTicks;
int16_t rightSignedTicks;

// previous absolute ticks
uint16_t previousRightEncoder;
uint16_t previousLeftEncoder;

int firstTicksMessage=1;

// odometry variables (x, y, theta)
float x, y, theta=0;
// constant parameters (baseline, right ticks per meter, left ticks per meter)
float kb, kr, kl;

// ros publishers
ros::Publisher odom_publisher;
ros::Publisher rel_ticks_publisher; // relative ticks publisher
ros::Publisher abs_ticks_publisher; // absolute ticks publisher

struct configuration {
    string robot_name;
    string serial_device;

    string subscribed_cmdvel_topic;
    string subscribed_ticks_topic;
    string published_rel_ticks_topic;
    string published_abs_ticks_topic;
    string published_odometry_topic;
    string published_link_name;
    string published_odom_link_name;

    string kbaseline;
    string kleft;
    string kright;

    int ascii;
    int heartbeat;
    int debug;
} c;

// Controll the robot in ticks
// Called when a tick massage is received from ros
// Forward the ticks to the serial port
void ticksCallback(const capybarauno::capybara_ticksConstPtr& ticks)
{
    if(!ros::ok()) return;
    speedPayload.leftTick=ticks->leftEncoder;
    speedPayload.rightTick=ticks->rightEncoder;
    //assign the payload to the general packet
    packet.speed=speedPayload;
    char buf[255];
    char* pEnd=Packet_write(&packet,buf,c.ascii);
    //send it
    sendToUart(serialFd,buf,pEnd-buf,0);
    if(c.debug)
        ROS_INFO("SENDING: %s\n",buf);
}

// Controll the reobot in cmdvel
void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    if(!ros::ok()) return;
    //get absolute speed values, expressed in tick per interval
    float translational_velocity =   twist->linear.x * 2;
    float rotational_velocity    = - twist->angular.z * kb;

    if(c.debug){
        ROS_INFO("LINEAR %f ANGULAR %f ANGULAR AFTER BASELINE %f", twist->linear.x, twist->angular.z,rotational_velocity);
    }

    speedPayload.leftTick = (-translational_velocity + rotational_velocity) / (kl * 2);
    speedPayload.rightTick = (translational_velocity + rotational_velocity) / (kr * 2);
    if(c.debug){
        ROS_INFO("TICKS %d %d",speedPayload.leftTick, speedPayload.rightTick);
    }
    //assign the payload to the general packet
    packet.speed=speedPayload;
    char buf[255];
    char* pEnd=Packet_write(&packet,buf,c.ascii);
    //send it
    sendToUart(serialFd,buf,pEnd-buf,0);
    if(c.debug)
        ROS_INFO("SENDING: %s\n",buf);
}

// read ticks from the serial port, convert them in relative signed int, then save and publish them.
void readTicksFromUartAndPublish( Packet_Decoder& decoder){
    char chunk;
    int complete=0;
    Packet read_packet;
    while(read(serialFd, &chunk, 1)>0 && !complete){
        complete = Packet_Decoder_putChar(&decoder,(unsigned char)chunk);
    }
    if(complete){
        Packet_parse(decoder.buffer_start,&read_packet,c.ascii);
    }

    if(firstTicksMessage){
        previousLeftEncoder = read_packet.state.leftEncoder;
        previousRightEncoder = read_packet.state.rightEncoder;
    }else{
        // needed to compute odometry and publish the relative signed int ticks.
        leftSignedTicks = -(int16_t)(read_packet.state.leftEncoder - previousLeftEncoder);
        rightSignedTicks = (int16_t)(read_packet.state.rightEncoder - previousRightEncoder);

        previousLeftEncoder =read_packet.state.leftEncoder;
        previousRightEncoder = read_packet.state.rightEncoder;

        // publish relative ticks
        capybarauno::capybara_ticks_signed ct;
        ct.leftEncoder=leftSignedTicks;
        ct.rightEncoder=rightSignedTicks;
        ct.header.stamp=ros::Time::now();
        ct.header.seq=read_packet.seq + 1;
        rel_ticks_publisher.publish(ct);

    }
    capybarauno::capybara_ticks ticks_message;
    ticks_message.header.seq = read_packet.seq;
    ticks_message.header.stamp = ros::Time::now();
    ticks_message.leftEncoder = read_packet.state.leftEncoder;
    ticks_message.rightEncoder = read_packet.state.rightEncoder;
    abs_ticks_publisher.publish(ticks_message);
}

void computeOdometryAndPublish(tf::TransformBroadcaster& odom_broadcaster) {
    if(!ros::ok()) return;

    float lt=leftSignedTicks*kl;
    float rt=rightSignedTicks*kr;

    float linear_vel = (lt + rt) / 2;
    theta -= (rt - lt) / kb;

    if(c.debug)
        ROS_INFO("lt: %f rt: %f theta: %f\n", lt, rt, theta);

    x += linear_vel*cos(theta);
    y += linear_vel*sin(theta);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    ros::Time current_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = c.published_odometry_topic.c_str();
    odom_trans.child_frame_id = c.published_link_name.c_str();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = c.published_odom_link_name;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom_publisher.publish(odom);
}

void robotCommunication_init(){
    initConsts();
    Packet_Decoder_init(&packet_decoder,c.ascii);
    packet.id=Speed_Payload_ID;
    serialFd=openPort((char*)c.serial_device.c_str());
    if(c.debug)
        ROS_INFO("serial port status: %d\n",serialFd);
}

void send_heartbeat(int &cnt){
    if(cnt%2==0){
        heartbeat_packet.seq++;
        char heartbuff[255];
        char* pEnd=Packet_write(&heartbeat_packet,heartbuff,c.ascii);
        sendToUart(serialFd,heartbuff,pEnd-heartbuff,0);
        if(c.debug)
            ROS_INFO("SENDING: %s\n",heartbuff);
    }
    cnt++;
}

void echoRosParameters(){
    ROS_INFO("Running with parameters:");
    ROS_INFO("%s %s\n","_robot_name:",               c.robot_name.c_str());
    ROS_INFO("%s %s\n","_serial_device:",            c.serial_device.c_str());

    ROS_INFO("%s %s\n","_subscribed_cmdvel_topic:",  c.subscribed_cmdvel_topic.c_str());
    ROS_INFO("%s %s\n","_subscribed_ticks_topic:",   c.subscribed_ticks_topic.c_str());
    ROS_INFO("%s %s\n","_published_rel_ticks_topic:",c.published_rel_ticks_topic.c_str());
    ROS_INFO("%s %s\n","_published_abs_ticks_topic:",c.published_abs_ticks_topic.c_str());
    ROS_INFO("%s %s\n","_published_odometry_topic:", c.published_odometry_topic.c_str());
    ROS_INFO("%s %s\n","_published_link_name:",      c.published_link_name.c_str());
    ROS_INFO("%s %s\n","_published_odom_link_name:", c.published_odom_link_name.c_str());

    ROS_INFO("%s %s\n","_kbaseline:",                c.kbaseline.c_str());
    ROS_INFO("%s %s\n","_kleft:",                    c.kleft.c_str());
    ROS_INFO("%s %s\n","_kright:",                   c.kright.c_str());

    ROS_INFO("%s %d\n","_ascii:",                    c.ascii);
    ROS_INFO("%s %d\n","_hearbeat:",                 c.heartbeat);
    ROS_INFO("%s %d\n","_debug:",                    c.debug);
}

void setRosParameters(ros::NodeHandle n){
    n.param<string>("robot_name", c.robot_name, "");
    n.param<string>("serial_device", c.serial_device, "/dev/ttyACM0");

    n.param<string>("subscribed_cmdvel_topic",  c.subscribed_cmdvel_topic, "/cmd_vel");
    n.param<string>("subscribed_ticks_topic",   c.subscribed_ticks_topic, "/requested_ticks");
    n.param<string>("published_rel_ticks_topic",c.published_rel_ticks_topic, "/relative_signed_ticks");
    n.param<string>("published_abs_ticks_topic",c.published_abs_ticks_topic, "/absolute_usigned_ticks");
    n.param<string>("published_odometry_topic", c.published_odometry_topic, "/odom");
    n.param<string>("published_link_name",      c.published_link_name, "/base_link");
    n.param<string>("published_odom_link_name", c.published_odom_link_name, "/odom");

    c.subscribed_cmdvel_topic = c.robot_name + c.subscribed_cmdvel_topic;
    c.subscribed_ticks_topic = c.robot_name + c.subscribed_ticks_topic;
    c.published_rel_ticks_topic = c.robot_name + c.published_rel_ticks_topic;
    c.published_abs_ticks_topic = c.robot_name + c.published_abs_ticks_topic;
    c.published_odometry_topic = c.robot_name + c.published_odometry_topic;
    c.published_link_name = c.robot_name + c.published_link_name;
    c.published_odom_link_name = c.robot_name + c.published_odom_link_name;

    n.param<string>("kbaseline",c.kbaseline, "0.2f");
    n.param<string>("kleft",    c.kleft, "0.001f");
    n.param<string>("kright",   c.kright, "0.001f");

    n.param<int>("ascii",     c.ascii, 1);
    n.param<int>("heartbeat", c.heartbeat, 1);
    n.param("debug",          c.debug, 1);
}

void initHeartbeatVariables(){
    heartbeat.beat=1;
    heartbeat_packet.id=Heartbeat_Payload_ID;
    heartbeat_packet.seq=0;
    heartbeat_packet.heartbeat=heartbeat;
}
void initOdometryVar(){
    previousLeftEncoder=0;
    previousRightEncoder=0;
    leftSignedTicks=0;
    rightSignedTicks=0;

    kb = atof(c.kbaseline.c_str());
    kr = atof(c.kright.c_str());
    kl = atof(c.kleft.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capybarauno_node",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    setRosParameters( n );
    echoRosParameters();

    initHeartbeatVariables();
    initOdometryVar();

    robotCommunication_init();

    ros::Subscriber ticks_subscriber  = n.subscribe(c.subscribed_ticks_topic.c_str(), 1000, ticksCallback);
    ros::Subscriber cmdvel_subscriber = n.subscribe(c.subscribed_cmdvel_topic.c_str(), 1000, cmdvelCallback);

    tf::TransformBroadcaster odom_broadcaster;
    odom_publisher      = n.advertise<nav_msgs::Odometry>(c.published_odometry_topic.c_str(), 1000);
    rel_ticks_publisher = n.advertise<capybarauno::capybara_ticks_signed>(c.published_rel_ticks_topic.c_str(), 1000);
    abs_ticks_publisher = n.advertise<capybarauno::capybara_ticks>(c.published_abs_ticks_topic.c_str(), 1000);

    //ros::ok() used to get the SIGINT ctrl+c
    while(ros::ok()){
        readTicksFromUartAndPublish(packet_decoder);

        computeOdometryAndPublish(odom_broadcaster);

        if(c.heartbeat==1)
            send_heartbeat(beatcnt);

        ros::spinOnce();
        usleep(1000);
    }

    return 0;
}
