// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/WrenchStamped.h>

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

ros::Publisher com_pub, rf_pub, lf_pub, rh_pub, lh_pub, h2r_ratio_pub;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    geometry_msgs::PointStamped msg;
	XnSkeletonJointPosition joint_position;
    msg.header.frame_id = "camera_link";//ホントは違う
    msg.header.stamp = ros::Time::now();
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(users[0], XN_SKEL_TORSO, joint_position);
	msg.point.x = -joint_position.position.Z / 1000.0;
	msg.point.y = joint_position.position.X / 1000.0;
	msg.point.z = joint_position.position.Y / 1000.0;
    com_pub.publish(msg);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(users[0], XN_SKEL_RIGHT_FOOT, joint_position);
	msg.point.x = -joint_position.position.Z / 1000.0;
	msg.point.y = joint_position.position.X / 1000.0;
	msg.point.z = joint_position.position.Y / 1000.0;
    rf_pub.publish(msg);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(users[0], XN_SKEL_LEFT_FOOT, joint_position);
	msg.point.x = -joint_position.position.Z / 1000.0;
	msg.point.y = joint_position.position.X / 1000.0;
	msg.point.z = joint_position.position.Y / 1000.0;
    lf_pub.publish(msg);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(users[0], XN_SKEL_LEFT_HAND, joint_position);//何故か逆
	msg.point.x = -joint_position.position.Z / 1000.0;
	msg.point.y = joint_position.position.X / 1000.0;
	msg.point.z = joint_position.position.Y / 1000.0;
    rh_pub.publish(msg);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(users[0], XN_SKEL_RIGHT_HAND, joint_position);
	msg.point.x = -joint_position.position.Z / 1000.0;
	msg.point.y = joint_position.position.X / 1000.0;
	msg.point.z = joint_position.position.Y / 1000.0;
    lh_pub.publish(msg);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}


double rfw[6],lfw[6];

void onZMPCB(const geometry_msgs::PointStampedConstPtr& msg) {
    std::cout<<"/zmp_robot"<<msg->point.x<<" , "<<msg->point.y<<std::endl;//BODY座標
    

    double rfzmp[2],lfzmp[2];
    double zmpans[3];//世界座標
    const double F_H_OFFSET = 0.03;
    const double rfpos[3] = {0,-0.1,0},lfpos[3] = {0,0.1,0};

    if( rfw[2] > 1.0e-6 ){
	    rfzmp[0] = ( -rfw[4] - rfw[0] * F_H_OFFSET + rfw[2] * 0 ) / rfw[2] + rfpos[0];
	    rfzmp[1] = ( rfw[3] - rfw[1] * F_H_OFFSET + rfw[2] * 0 ) / rfw[2] + rfpos[1];
    }
    if( lfw[2] > 1.0e-6 ){
	    lfzmp[0] = ( -lfw[4] - lfw[0] * F_H_OFFSET + lfw[2] * 0 ) / lfw[2] + lfpos[0];
	    lfzmp[1] = ( lfw[3] - lfw[1] * F_H_OFFSET + lfw[2] * 0 ) / lfw[2] + lfpos[1];
    }
    //zmpansをrfw,lfwから計算
    if( rfw[2] > 1.0e-6 || lfw[2] > 1.0e-6 ){
	    zmpans[0] = ( rfzmp[0]*rfw[2] + lfzmp[0]*lfw[2] ) / ( rfw[2] + lfw[2]);
	    zmpans[1] = ( rfzmp[1]*rfw[2] + lfzmp[1]*lfw[2] ) / ( rfw[2] + lfw[2]);
	    zmpans[2] = 0;
    }else{
	    zmpans[0] = 0;	zmpans[1] = 0;	zmpans[2] = 0;
    }
    std::cout<<"/zmp_human"<<zmpans[0]<<" , "<<zmpans[1]<<std::endl;//BODY座標

    
}
void onRFWCB(const geometry_msgs::WrenchStampedConstPtr& msg) {
	rfw[0] = msg->wrench.force.x;	rfw[1] = msg->wrench.force.y;	rfw[2] = msg->wrench.force.z;
	rfw[3] = msg->wrench.torque.x;	rfw[4] = msg->wrench.torque.y;	rfw[5] = msg->wrench.torque.z;
}
void onLFWCB(const geometry_msgs::WrenchStampedConstPtr& msg) {
	lfw[0] = msg->wrench.force.x;	lfw[1] = msg->wrench.force.y;	lfw[2] = msg->wrench.force.z;
	lfw[3] = msg->wrench.torque.x;	lfw[4] = msg->wrench.torque.y;	lfw[5] = msg->wrench.torque.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;
	com_pub = nh.advertise<geometry_msgs::PointStamped>("/human_tracker_com_ref", 10);
	rf_pub = nh.advertise<geometry_msgs::PointStamped>("/human_tracker_rf_ref", 10);
	lf_pub = nh.advertise<geometry_msgs::PointStamped>("/human_tracker_lf_ref", 10);
	rh_pub = nh.advertise<geometry_msgs::PointStamped>("/human_tracker_rh_ref", 10);
	lh_pub = nh.advertise<geometry_msgs::PointStamped>("/human_tracker_lh_ref", 10);
	h2r_ratio_pub = nh.advertise<std_msgs::Int32>("/human_tracker_h2r_ratio", 10);
	
    ros::Subscriber zmp_sub = nh.subscribe("/zmp", 1, &onZMPCB);
    ros::Subscriber rfw_sub = nh.subscribe("/human_tracker_rfw_ref", 1, &onRFWCB);
    ros::Subscriber lfw_sub = nh.subscribe("/human_tracker_lfw_ref", 1, &onLFWCB);

    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
	    if (nRetVal != XN_STATUS_OK) {
		    ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
	    }
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

        
        ros::NodeHandle pnh("~");
        string frame_id("openni_depth_frame");
        pnh.getParam("camera_frame_id", frame_id);
                
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
