#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>

ros::Publisher bbox_pub, yoloBBoxesPub_;
vision_msgs::Detection2DArray yoloDetectionResults_; // yolo detected 2D results
cv::Mat alignedDepthImage_;
cv::Mat detectedAlignedDepthImg_;

double depthScale_ = 1000.0;
int depthFilterMargin_ = 10;
int imgRows_ = 480;
int imgCols_ = 640;
double depthMinValue_ = 0.2;
double depthMaxValue_ = 5.0;
bool odom_sub = false;

// std::vector<double> colorIntrinsics = {604.404296875, 604.404296875, 325.03704833984375, 245.77059936523438};
std::vector<double> colorIntrinsics = {554.254691191187, 554.254691191187, 320.5, 240.5}; // for xtdeone

double fxC_, fyC_, cxC_, cyC_;

Eigen::Vector3d position_;         // depth camera position
Eigen::Matrix3d orientation_;      // depth camera orientation
Eigen::Vector3d positionColor_;    // color camera position
Eigen::Matrix3d orientationColor_; // color camera orientation

Eigen::Matrix4d body2Cam_;

Eigen::Matrix4d body2CamColor_;

const Eigen::Vector3d humanSize(0.5, 0.5, 1.8);

struct box3D
{
    float x, y, z;
    float x_width, y_width, z_width;
    float id;
    float Vx, Vy;
    float Ax, Ay;
    bool is_human = false;   // false: not detected by yolo as dynamic, true: detected by yolo
    bool is_dynamic = false; // false: not detected as dynamic(either yolo or classificationCB), true: detected as dynamic
    bool fix_size = false;   // flag to force future boxes to fix size
    bool is_dynamic_candidate = false;
};

void calculateMAD(std::vector<double> &depthValues, double &depthMedian, double &MAD)
{
    std::sort(depthValues.begin(), depthValues.end());
    int medianIdx = int(depthValues.size() / 2);
    depthMedian = depthValues[medianIdx]; // median of all data

    std::vector<double> deviations;
    for (size_t i = 0; i < depthValues.size(); ++i)
    {
        deviations.push_back(std::abs(depthValues[i] - depthMedian));
    }
    std::sort(deviations.begin(), deviations.end());
    MAD = deviations[int(deviations.size() / 2)];
}

void transformBBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size, const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation,
                   Eigen::Vector3d &newCenter, Eigen::Vector3d &newSize)
{
    double x = center(0);
    double y = center(1);
    double z = center(2);
    double xWidth = size(0);
    double yWidth = size(1);
    double zWidth = size(2);

    // get 8 bouding boxes coordinates in the camera frame
    Eigen::Vector3d p1(x + xWidth / 2.0, y + yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p2(x + xWidth / 2.0, y + yWidth / 2.0, z - zWidth / 2.0);
    Eigen::Vector3d p3(x + xWidth / 2.0, y - yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p4(x + xWidth / 2.0, y - yWidth / 2.0, z - zWidth / 2.0);
    Eigen::Vector3d p5(x - xWidth / 2.0, y + yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p6(x - xWidth / 2.0, y + yWidth / 2.0, z - zWidth / 2.0);
    Eigen::Vector3d p7(x - xWidth / 2.0, y - yWidth / 2.0, z + zWidth / 2.0);
    Eigen::Vector3d p8(x - xWidth / 2.0, y - yWidth / 2.0, z - zWidth / 2.0);

    // transform 8 points to the map coordinate frame
    Eigen::Vector3d p1m = orientation * p1 + position;
    Eigen::Vector3d p2m = orientation * p2 + position;
    Eigen::Vector3d p3m = orientation * p3 + position;
    Eigen::Vector3d p4m = orientation * p4 + position;
    Eigen::Vector3d p5m = orientation * p5 + position;
    Eigen::Vector3d p6m = orientation * p6 + position;
    Eigen::Vector3d p7m = orientation * p7 + position;
    Eigen::Vector3d p8m = orientation * p8 + position;
    std::vector<Eigen::Vector3d> pointsMap{p1m, p2m, p3m, p4m, p5m, p6m, p7m, p8m};

    // find max min in x, y, z directions
    double xmin = p1m(0);
    double xmax = p1m(0);
    double ymin = p1m(1);
    double ymax = p1m(1);
    double zmin = p1m(2);
    double zmax = p1m(2);
    for (Eigen::Vector3d pm : pointsMap)
    {
        if (pm(0) < xmin)
        {
            xmin = pm(0);
        }
        if (pm(0) > xmax)
        {
            xmax = pm(0);
        }
        if (pm(1) < ymin)
        {
            ymin = pm(1);
        }
        if (pm(1) > ymax)
        {
            ymax = pm(1);
        }
        if (pm(2) < zmin)
        {
            zmin = pm(2);
        }
        if (pm(2) > zmax)
        {
            zmax = pm(2);
        }
    }
    newCenter(0) = (xmin + xmax) / 2.0;
    newCenter(1) = (ymin + ymax) / 2.0;
    newCenter(2) = (zmin + zmax) / 2.0;
    newSize(0) = xmax - xmin;
    newSize(1) = ymax - ymin;
    newSize(2) = zmax - zmin;
}

void getCameraPose(const nav_msgs::OdometryConstPtr &odom, Eigen::Matrix4d &camPoseMatrix, Eigen::Matrix4d &camPoseColorMatrix)
{
    Eigen::Quaterniond quat;
    quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    Eigen::Matrix3d rot = quat.toRotationMatrix();

    // convert body pose to camera pose
    Eigen::Matrix4d map2body;
    map2body.setZero();
    map2body.block<3, 3>(0, 0) = rot;
    map2body(0, 3) = odom->pose.pose.position.x;
    map2body(1, 3) = odom->pose.pose.position.y;
    map2body(2, 3) = odom->pose.pose.position.z;
    map2body(3, 3) = 1.0;

    camPoseMatrix = map2body * body2Cam_;
    camPoseColorMatrix = map2body * body2CamColor_;
}

void publish3dBox(const std::vector<box3D> &boxes, const ros::Publisher &publisher, double r, double g, double b)
{
    // visualization using bounding boxes
    visualization_msgs::Marker line;
    visualization_msgs::MarkerArray lines;
    line.header.frame_id = "world";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.ns = "box3D";
    line.scale.x = 0.06;
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;
    line.lifetime = ros::Duration(0.1);

    for (size_t i = 0; i < boxes.size(); i++)
    {
        // visualization msgs
        line.text = " Vx " + std::to_string(boxes[i].Vx) + " Vy " + std::to_string(boxes[i].Vy);
        double x = boxes[i].x;
        double y = boxes[i].y;
        double z = (boxes[i].z + boxes[i].z_width / 2) / 2;

        // double x_width = std::max(boxes[i].x_width,boxes[i].y_width);
        // double y_width = std::max(boxes[i].x_width,boxes[i].y_width);
        double x_width = boxes[i].x_width;
        double y_width = boxes[i].y_width;
        double z_width = 2 * z;

        // double z =

        std::vector<geometry_msgs::Point> verts;
        geometry_msgs::Point p;
        // vertice 0
        p.x = x - x_width / 2.;
        p.y = y - y_width / 2.;
        p.z = z - z_width / 2.;
        verts.push_back(p);

        // vertice 1
        p.x = x - x_width / 2.;
        p.y = y + y_width / 2.;
        p.z = z - z_width / 2.;
        verts.push_back(p);

        // vertice 2
        p.x = x + x_width / 2.;
        p.y = y + y_width / 2.;
        p.z = z - z_width / 2.;
        verts.push_back(p);

        // vertice 3
        p.x = x + x_width / 2.;
        p.y = y - y_width / 2.;
        p.z = z - z_width / 2.;
        verts.push_back(p);

        // vertice 4
        p.x = x - x_width / 2.;
        p.y = y - y_width / 2.;
        p.z = z + z_width / 2.;
        verts.push_back(p);

        // vertice 5
        p.x = x - x_width / 2.;
        p.y = y + y_width / 2.;
        p.z = z + z_width / 2.;
        verts.push_back(p);

        // vertice 6
        p.x = x + x_width / 2.;
        p.y = y + y_width / 2.;
        p.z = z + z_width / 2.;
        verts.push_back(p);

        // vertice 7
        p.x = x + x_width / 2.;
        p.y = y - y_width / 2.;
        p.z = z + z_width / 2.;
        verts.push_back(p);

        int vert_idx[12][2] = {
            {0, 1},
            {1, 2},
            {2, 3},
            {0, 3},
            {0, 4},
            {1, 5},
            {3, 7},
            {2, 6},
            {4, 5},
            {5, 6},
            {4, 7},
            {6, 7}};

        for (size_t i = 0; i < 12; i++)
        {
            line.points.push_back(verts[vert_idx[i][0]]);
            line.points.push_back(verts[vert_idx[i][1]]);
        }

        lines.markers.push_back(line);

        line.id++;
    }
    // publish
    publisher.publish(lines);
}

void alignedDepthCB(const sensor_msgs::ImageConstPtr &img)
{

    cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (imgPtr->image).convertTo(imgPtr->image, CV_16UC1, depthScale_);
    }
    imgPtr->image.copyTo(alignedDepthImage_);

    cv::Mat depthNormalized;
    imgPtr->image.copyTo(depthNormalized);
    double min, max;
    cv::minMaxIdx(depthNormalized, &min, &max);
    cv::convertScaleAbs(depthNormalized, depthNormalized, 255. / max);
    depthNormalized.convertTo(depthNormalized, CV_8UC1);
    cv::applyColorMap(depthNormalized, depthNormalized, cv::COLORMAP_BONE);
    detectedAlignedDepthImg_ = depthNormalized;
}

void OdomCB(const nav_msgs::OdometryConstPtr &odom)
{
    odom_sub = true;
    // store current position and orientation (camera)
    Eigen::Matrix4d camPoseMatrix, camPoseColorMatrix;
    getCameraPose(odom, camPoseMatrix, camPoseColorMatrix);

    position_(0) = camPoseMatrix(0, 3);
    position_(1) = camPoseMatrix(1, 3);
    position_(2) = camPoseMatrix(2, 3);
    orientation_ = camPoseMatrix.block<3, 3>(0, 0);

    positionColor_(0) = camPoseColorMatrix(0, 3);
    positionColor_(1) = camPoseColorMatrix(1, 3);
    positionColor_(2) = camPoseColorMatrix(2, 3);
    orientationColor_ = camPoseColorMatrix.block<3, 3>(0, 0);
}

void detection_callback(const vision_msgs::Detection2DArrayConstPtr &detections)
{
    if (alignedDepthImage_.empty() || !odom_sub)
    {
        ROS_WARN("No Image or Odom!");
        return;
    }

    if (detections->detections.empty())
    {
        // ROS_WARN("No detections available.");
        return;
    }

    yoloDetectionResults_ = *detections;
    std::vector<box3D> yoloBBoxesTemp;

    for (size_t i = 0; i < yoloDetectionResults_.detections.size(); ++i)
    {
        vision_msgs::Detection2D detection;
        box3D bbox3D;
        detection = yoloDetectionResults_.detections[i];

        // 1. retrive 2D detection result
        int topX = int(detection.bbox.center.x);
        int topY = int(detection.bbox.center.y);
        int xWidth = int(detection.bbox.size_x);
        int yWidth = int(detection.bbox.size_y);
        // ROS_INFO("Detection %zu: topX = %d, topY = %d, xWidth = %d, yWidth = %d", i, topX, topY, xWidth, yWidth);
        // 2. get thickness estimation (double MAD: double Median Absolute Deviation)
        uint16_t *rowPtr;
        double depth;
        const double inv_factor = 1.0 / depthScale_;
        // const double inv_factor = 1.0 ;
        int vMin = std::min(topY, depthFilterMargin_);
        int uMin = std::min(topX, depthFilterMargin_);
        int vMax = std::min(topY + yWidth, imgRows_ - depthFilterMargin_);
        int uMax = std::min(topX + xWidth, imgCols_ - depthFilterMargin_);

        // ROS_INFO("Detection %zu: vMin = %d, uMin = %d, vMax = %d, uMax = %d", i, vMin, uMin, vMax, uMax);

        std::vector<double> depthValues;
        // record the depth values in the potential regions
        for (int v = vMin; v < vMax; ++v)
        { // row
            rowPtr = alignedDepthImage_.ptr<uint16_t>(v);
            for (int u = uMin; u < uMax; ++u)
            { // column
                depth = (*rowPtr) * inv_factor;
                // ROS_INFO("Depth at (v: %d, u: %d) = %f", v, u, depth);
                if (depth >= depthMinValue_ and depth <= depthMaxValue_)
                {
                    depthValues.push_back(depth);
                }
                ++rowPtr;
            }
        }
        if (depthValues.size() == 0)
        { // in case of out of range
            return;
        }
        // ROS_WARN("Get all depth");
        // double MAD calculation
        double depthMedian, MAD;
        calculateMAD(depthValues, depthMedian, MAD);
        // std::cout << "MAD: " << MAD << std::endl;

        double depthMin = 10.0;
        double depthMax = -10.0;
        // find min max depth value
        for (int v = vMin; v < vMax; ++v)
        { // row
            rowPtr = alignedDepthImage_.ptr<uint16_t>(v);
            for (int u = uMin; u < uMax; ++u)
            { // column
                depth = (*rowPtr) * inv_factor;
                if (depth >= depthMinValue_ and depth <= depthMaxValue_)
                {
                    if ((depth < depthMin) and (depth >= depthMedian - 1.5 * MAD))
                    {
                        depthMin = depth;
                    }

                    if ((depth > depthMax) and (depth <= depthMedian + 1.5 * MAD))
                    {
                        depthMax = depth;
                    }
                }
                ++rowPtr;
            }
        }

        if (depthMin == 10.0 or depthMax == -10.0)
        { // in case the depth value is not available
            return;
        }
        // ROS_WARN("depthMin and depthMax");

        // 3. project points into 3D in the camera frame
        Eigen::Vector3d pUL, pBR, center;
        pUL(0) = (topX - cxC_) * depthMedian / fxC_;
        pUL(1) = (topY - cyC_) * depthMedian / fyC_;
        pUL(2) = depthMedian;

        pBR(0) = (topX + xWidth - cxC_) * depthMedian / fxC_;
        pBR(1) = (topY + yWidth - cyC_) * depthMedian / fyC_;
        pBR(2) = depthMedian;

        center(0) = (pUL(0) + pBR(0)) / 2.0;
        center(1) = (pUL(1) + pBR(1)) / 2.0;
        center(2) = depthMedian;

        double xWidth3D = std::abs(pBR(0) - pUL(0));
        double yWidth3D = std::abs(pBR(1) - pUL(1));
        double zWidth3D = depthMax - depthMin;
        if ((zWidth3D / humanSize(2) >= 2.0) or (zWidth3D / humanSize(2) <= 0.5))
        { // error is too large, then use the predefined size
            zWidth3D = humanSize(2);
        }
        Eigen::Vector3d size(xWidth3D, yWidth3D, zWidth3D);

        // 4. transform 3D points into world frame
        Eigen::Vector3d newCenter, newSize;
        transformBBox(center, size, positionColor_, orientationColor_, newCenter, newSize);
        bbox3D.x = newCenter(0);
        bbox3D.y = newCenter(1);
        bbox3D.z = newCenter(2);

        bbox3D.x_width = newSize(0);
        bbox3D.y_width = newSize(1);
        bbox3D.z_width = newSize(2);

        // 5. check the bounding box size. If the bounding box size is too different from the predefined size, overwrite the size
        if ((bbox3D.x_width / humanSize(0) >= 2.0) or (bbox3D.x_width / humanSize(0) <= 0.5))
        {
            bbox3D.x_width = humanSize(0);
        }

        if ((bbox3D.y_width / humanSize(1) >= 2.0) or (bbox3D.y_width / humanSize(1) <= 0.5))
        {
            bbox3D.y_width = humanSize(1);
        }

        if ((bbox3D.z_width / humanSize(2) >= 2.0) or (bbox3D.z_width / humanSize(2) <= 0.5))
        {
            bbox3D.z = humanSize(2) / 2.;
            bbox3D.z_width = humanSize(2);
        }
        yoloBBoxesTemp.push_back(bbox3D);
        // 6. Publishing three-dimensional positions
        geometry_msgs::Point target_position;
        target_position.x = bbox3D.x;
        target_position.y = bbox3D.y;
        target_position.z = 1.0; // person dont move in z-aixs

        bbox_pub.publish(target_position);
    }
    publish3dBox(yoloBBoxesTemp, yoloBBoxesPub_, 1, 0, 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_target_ekf");
    ros::NodeHandle nh("~");

    fxC_ = colorIntrinsics[0];
    fyC_ = colorIntrinsics[1];
    cxC_ = colorIntrinsics[2];
    cyC_ = colorIntrinsics[3];
    body2Cam_ << 0.0, 0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    body2CamColor_ << 0.0, 0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    ros::Subscriber det_sub = nh.subscribe("/yolo_detector/detected_bounding_boxes", 100, detection_callback);
    ros::Subscriber alignedDepthSub_ = nh.subscribe("/iris_0/realsense/depth_camera/depth/image_raw", 10, alignedDepthCB);
    ros::Subscriber OdomSub = nh.subscribe("/iris_0/mavros/vision_odom/odom", 30, OdomCB);
    bbox_pub = nh.advertise<geometry_msgs::Point>("/target_3d_position", 10);
    yoloBBoxesPub_ = nh.advertise<visualization_msgs::MarkerArray>("/yolo_3d_bboxes", 10);
    ros::spin();
    return 0;
}