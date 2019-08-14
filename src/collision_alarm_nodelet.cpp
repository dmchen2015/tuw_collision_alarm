#include <collision_alarm_nodelet.h>

namespace tuw_collision_alarm {

    void CollisionAlarmNodelet::onInit() {

        NODELET_INFO ("Initializing nodelet CollisionAlarmNodelet...");

        nh_ = getNodeHandle();
        private_nh_ = getPrivateNodeHandle();
        std::string s1;

        //nh_.param< std::string >("distance_threshold",s1,"0.2");
        private_nh_.param< int >("obstacle_vote_threshold",obstacleOnTheWayVoteThreshold, 5);
        private_nh_.param< double >("distance_threshold",distance_threshold, 0.5);
        NODELET_INFO("distance_threshold %lf meters",distance_threshold);
        NODELET_INFO("Number of laser points for something to be considered an obstacle: %i",obstacleOnTheWayVoteThreshold);

        WaypointArrayLastIndexBehind = 0;
        WaypointArrayLastIndexFront = -1;
        abruptJumpChecker = 0;

        sub_laser_ = nh_.subscribe("laser0/scan", 10, &CollisionAlarmNodelet::callbackLaser, this);
        sub_path_ = nh_.subscribe("global_planner/planner/plan", 1, &CollisionAlarmNodelet::callbackPath, this);
        timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&CollisionAlarmNodelet::callbackTimer, this, _1));
        pub_waypoint_index = nh_.advertise<std_msgs::Int32>("collision_alarm_index",1);


    }

    void CollisionAlarmNodelet::callbackLaser(const sensor_msgs::LaserScan::ConstPtr &input_scan) {

        laserEndPointsPtr = std::make_shared<std::vector<tuw::Point2D>>();
        laserScanPtr_ = input_scan;
        for (size_t j = 0; j < laserScanPtr_->ranges.size(); j++) {
            if (std::isnan(laserScanPtr_->ranges[j])) { //skip the NaN measurements
                continue;
            }
            laserEndPointsPtr->push_back(calculateLaserEndpoints(j)); // already transformed to robot coordinates

        }


    }

    void CollisionAlarmNodelet::callbackPath(const nav_msgs::Path::ConstPtr &poseArray) {
        //making the assumption that the Poses are ordered in such a way that the first is the closest to the robot
        waypointsPtr_ = poseArray;
        WaypointArrayLastIndexBehind = 0; // start allover again when there is a new waypoint array
        WaypointArrayLastIndexFront = -1;


    }



    tuw::Point2D CollisionAlarmNodelet::calculateLaserEndpoints(size_t laserScanIndex) {
        double beamAngle = laserScanPtr_->angle_min + (laserScanPtr_->angle_increment * laserScanIndex);
        double endPointX = cos(beamAngle) * laserScanPtr_->ranges[laserScanIndex];
        double endPointY = sin(beamAngle) * laserScanPtr_->ranges[laserScanIndex];
        tf::Vector3 point(endPointX, endPointY, 0);
        tf::Vector3 point_bl = tftransform * point;
        return tuw::Point2D(point_bl.getX(), point_bl.getY());
    }


    void CollisionAlarmNodelet::callbackTimer(const ros::TimerEvent &event) {
        try {
            tflistener_.lookupTransform("r0/base_link", "r0/laser0", ros::Time(0), tftransform); // seems correct
            tflistener_.lookupTransform("r0/base_link", "map", ros::Time(0), tftransform2);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        if (waypointsPtr_ == nullptr || laserScanPtr_ == nullptr) {
            return;
        }

        filterWaypoints(waypointsPtr_);


    }

    void CollisionAlarmNodelet::filterWaypoints(const nav_msgs::Path::ConstPtr &poseArray) {
        //assuming that i got the transfrom and a path
        // transform the path 1 by one, dont do it beforehand
        abruptJumpChecker = WaypointArrayLastIndexBehind;
        size_t obstacleOnTheWayVote = 0;
        tf::Transform temp_tf_object;
        geometry_msgs::Pose poseN0;
        geometry_msgs::Pose poseN1;
        size_t i = 0;
        for (i = WaypointArrayLastIndexBehind;
             i < poseArray->poses.size() - 1; i++) { // -1 cuz, N elements, 0-1, 1-2 ... (N-2) - ( N-1)
            tf::poseMsgToTF(poseArray->poses[i].pose, temp_tf_object);
            tf::poseTFToMsg(tftransform2 * temp_tf_object, poseN0);

            if (poseN0.position.x < 0.1) { //check if its behind the robot i.e x<0
                WaypointArrayLastIndexBehind = i; // save this index to start from this next time
                continue; // go the next iteration cuz we are not interested

            } else if (poseN0.position.x > (laserScanPtr_->range_max)) {
                NODELET_INFO ("BREAKING DUE TO LASER SCAN RANGE REACHED...");
                WaypointArrayLastIndexFront = i; // save this to check timely loop ending
                break; //no need to check any further, we are beyond the lasers scan range

            } else {
                tf::poseMsgToTF(poseArray->poses[i + 1].pose, temp_tf_object);
                tf::poseTFToMsg(tftransform2 * temp_tf_object, poseN1); // transform the next waypoint
                tuw::Point2D wayPoint0(poseN0.position.x, poseN0.position.y);
                tuw::Point2D wayPoint1(poseN1.position.x, poseN1.position.y);
                tuw::LineSegment2D lineSegment(wayPoint0, wayPoint1); //create the line segment
                for (const auto &laserEndPoint: *laserEndPointsPtr) {
                    double distance_to_line = lineSegment.distanceTo(laserEndPoint);
                    if (distance_to_line < distance_threshold) {
                        obstacleOnTheWayVote += 1;
                    }
                }
                NODELET_INFO("Iterated the laser scan array, number of Obstacle on the way Votes = %lu",
                             obstacleOnTheWayVote);

            }
            if (obstacleOnTheWayVote > obstacleOnTheWayVoteThreshold) { //There is an obstacle close to the current
                break; //segment, no need to check any further
            } else {
                obstacleOnTheWayVote = 0;
            }
        }



        if ((abruptJumpChecker + 4) < WaypointArrayLastIndexBehind) {
            NODELET_INFO("ABRUPT JUMP!");
            WaypointArrayLastIndexBehind = abruptJumpChecker;
        }



        if (i == WaypointArrayLastIndexFront ||
            i == poseArray->poses.size() - 1) { // loop broke cuz we are past the laser range, -> all clear
            ROS_INFO("ALL CLEAR, SENDING THE LAST INDEX %lu...",i);
            WaypointArrayLastIndexFront = 0;
            std_msgs::Int32 index_msg;
            index_msg.data = poseArray->poses.size();
            pub_waypoint_index.publish(index_msg);


        } else { // there is an obstacle between waypoint i and i +1 , better send the i-1 as the last pose
            ROS_INFO("SOMETHING ON THE WAY BETWEEN %lu - %lu , SENDING THE INDEX %lu ", i, i + 1 ,i-1);
            std_msgs::Int32 index_msg;
            index_msg.data = i-1;
            pub_waypoint_index.publish(index_msg);

        }


    }


}


PLUGINLIB_EXPORT_CLASS (tuw_collision_alarm::CollisionAlarmNodelet, nodelet::Nodelet)
//int main(int argc, char **argv) {}