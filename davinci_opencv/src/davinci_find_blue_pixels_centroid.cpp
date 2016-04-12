//get an image, search for blue pixels;
// republish on new topic...and print out result
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <cwru_msg/vec_of_doubles.h>
#include <geometry_msgs/Point.h>
#include "cwru_opencv_common/projective_geometry.h"


static const std::string OPENCV_WINDOW = "Image window2";
using namespace std;

int g_blueratio;
double g_du_right,g_du_left,g_dv_right,g_dv_left;
const double width_in_pixels = 640;
const double height_in_pixels = 480;
const double baseline = 0.010;
const double f = 1034.0; // from horizontal fov = 0.6

class ImageConverter {

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_left_;
    image_transport::Subscriber image_sub_right_;    
    image_transport::Publisher image_pub_;

    float i_avg_left;
    float j_avg_left;
    float i_avg_right;
    float j_avg_right;

public:

    ImageConverter()
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_left_ = it_.subscribe("/davinci_endo/left/image_rect_color", 1,
                &ImageConverter::imageCbLeft, this);
        image_pub_ = it_.advertise("/image_converter/output_video2", 1);

        //extend to right camera:
        image_sub_right_ = it_.subscribe("/davinci_endo/right/image_rect_color", 1,
                &ImageConverter::imageCbRight, this);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCbLeft(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0;
        int isum = 0;
        int jsum = 0;
        int redval,blueval,greenval,testval;
        cv::Vec3b rgbpix;
        //image.at<uchar>(j,i)= 255;
        /**/
        for (int i = 0; i < cv_ptr->image.cols; i++)
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j,i); //[j][i];
                redval = rgbpix[2]+1;
                blueval = rgbpix[0]+1;
                greenval = rgbpix[1]+1;
                testval = blueval/(redval+greenval);
                //redval = (int) cv_ptr->image.at<cv::Vec3b>(j, i)[0]; 
                //cout<<"image("<<j<<","<<i<<")[0] = "<<redval<<endl;
                //if blue, paint it red:
                if (testval > g_blueratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++;
                    isum += i;
                    jsum += j;
                }
                else {
                    /*
                    for (int j = 0; j < cv_ptr->image.rows; j++) {
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 128;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    }
                     * */
                }
            }
         /* */
        cout << "npix: " << npix << endl;
        if (npix>0) {

            i_avg_left = (float) isum / (float) npix;
            i_avg_left = (float) jsum / (float) npix;
            cout << "i_avg: " << i_avg_left << endl;
            cout << "j_avg: " << j_avg_left << endl;

        }
        // g_du_left= (((double) (isum))/((double) npix))-width_in_pixels/2;
        // g_dv_left= (((double) (jsum))/((double) npix))-height_in_pixels/2;

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

    }
    
    void imageCbRight(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix_right = 0;
        int isum_right = 0;
        int jsum_right = 0;
        int redval,blueval,greenval,testval;
        cv::Vec3b rgbpix;
        //image.at<uchar>(j,i)= 255;
        /**/
        for (int i = 0; i < cv_ptr->image.cols; i++)
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j,i); //[j][i];
                redval = rgbpix[2]+1;
                blueval = rgbpix[0]+1;
                greenval = rgbpix[1]+1;
                testval = blueval/(redval+greenval);
                //redval = (int) cv_ptr->image.at<cv::Vec3b>(j, i)[0]; 
                //cout<<"image("<<j<<","<<i<<")[0] = "<<redval<<endl;
                //if blue, paint it red:
                if (testval > g_blueratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix_right++;
                    isum_right += i;
                    jsum_right += j;
                }
                else {
                    /*
                    for (int j = 0; j < cv_ptr->image.rows; j++) {
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 128;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    }
                     * */
                }
            }
         /* */
        cout << "npix_right: " << npix_right << endl;
        if (npix_right>0) {

            i_avg_right = (float) isum_right / (float) npix_right;
            j_avg_right = (float) jsum_right / (float) npix_right;
            cout << "i_avg: " << i_avg_right << endl;
            cout << "j_avg: " << j_avg_right << endl;


        }
        // g_du_right= (((double) (isum_right))/((double) npix_right))-width_in_pixels/2;
        // g_dv_right= (((double) (jsum_right))/((double) npix_right))-height_in_pixels/2;        

                
        // Update GUI Window
        //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //cv::waitKey(3);

        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());

    }

    void deprojectPoints(geometry_msgs::Point& centroid_coords_msg, const  cv::Mat &P_l , const cv::Mat &P_r)
    {
        cv_local::stereoCorrespondence stereo_correspond;

        cv::Point2f left_point(i_avg_left, j_avg_left);
        cv::Point2f right_point(i_avg_right, j_avg_right);

        stereo_correspond.left = left_point;
        stereo_correspond.right = right_point;

        cv::Point3d temp_point = cv_projective::deprojectStereoPoint(stereo_correspond, P_l , P_r);
        centroid_coords_msg.x = temp_point.x;
        centroid_coords_msg.y = temp_point.y;
        centroid_coords_msg.z = temp_point.z;
    }
};

    int main(int argc, char** argv) {
        ros::init(argc, argv, "blue_triangulator");
        ros::NodeHandle n; // 
        ros::Publisher pub = n.advertise<geometry_msgs::Point>("blue_centroid", 1);
        // instaniate a camera projection matrix object
        cv_projective::cameraProjectionMatrices cam_proj_mat(n, 
            std::string("/davinci_endo/left/camera_info"), std::string("/davinci_endo/right/camera_info"));
        cv::Mat P_l_mat; // define a cv::Mat variable to store left camera projection matrix
        cv::Mat P_r_mat; // define a cv::Mat variable to store right camera projection matrix
        
        ImageConverter imageConverter;
        // cwru_msg::vec_of_doubles centroid_coords_msg;
        geometry_msgs::Point centroid_coords_msg;
        // centroid_coords_msg.dvec.resize(3);
        cout<<"enter blue ratio threshold: (e.g. 10) ";
        cin>>g_blueratio;
        ros::Duration timer(0.1);
        double x,y,z;
        double disparity_u;
        //double baseline=0.010; //interocular distance
        //double f = 1032.0; // focal length, in pixels
        while(ros::ok()) {

            P_l_mat = cam_proj_mat.getLeftProjectionMatrix();
            P_r_mat = cam_proj_mat.getRightProjectionMatrix();
            imageConverter.deprojectPoints(centroid_coords_msg, P_l_mat, P_r_mat);
            // disparity_u = g_du_right-g_du_left;
            // cout<<"disparity in u: "<<disparity_u<<endl;
            // z = baseline*f/disparity_u;
            // x = 0.5*((g_du_right + g_du_left)/f)*z -baseline/2;
            // y = 0.5*((g_dv_right+g_dv_left)/f)*z;
            // ROS_INFO("x,y,z = %f, %f, %f",x,y,z);
            // centroid_coords_msg.dvec[0]=x;
            // centroid_coords_msg.dvec[1]=y;
            // centroid_coords_msg.dvec[2]=z;




            pub.publish(centroid_coords_msg);
            ros::spinOnce();
            timer.sleep();
        }
        return 0;
    }
