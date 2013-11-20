/*  
   bag_to_img
    A tool for saving images from a topic in a .bag file
    
    By Karl D. Hansen
    Aalborg University
    16-10-2012
*/

#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

// Title for the window showing the saved images
static const char WINDOW[] = "Saving image...";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_img");
    
    // Display usage help
    if (argc < 4) 
    {
        boost::filesystem::path program_path(argv[0]);
        std::cerr << "Usage: " << program_path.filename().string() << " BAGFILE TOPIC OUTPUTDIR" << std::endl;
        std::cerr << "Example: " << program_path.filename().string() << " data.bag /camera/image_raw ./images" << std::endl;
        return (-1);
    }

    // Open the bagfile
    rosbag::Bag bag;
    rosbag::View view;
    rosbag::View::iterator view_it;
    try
    {
        bag.open(argv[1], rosbag::bagmode::Read);
    } 
    catch (rosbag::BagException) 
    {
        ROS_FATAL_STREAM("Error opening file " << argv[1]);
        return(-1);
    }
    // Tell the bag that we will get the images
    view.addQuery (bag, rosbag::TopicQuery(argv[2]));

    // Create the directory for all the images
    std::string output_dir = std::string(argv[3]);
    boost::filesystem::path out_path(output_dir);
    if (!boost::filesystem::exists(out_path))
    {
        if (!boost::filesystem::create_directories(out_path))
        {
            std::cerr << "Error creating directory " << output_dir << std::endl;
            return(-1);
        }
        std::cout << "Creating directory " << output_dir << std::endl;
    }

    // Loop over the whole bag file
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
        if (i != NULL)
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                // Convert the ROS image to openCV mat-format
                cv_ptr = cv_bridge::toCvShare(i, i->encoding);
                // Save the image
                std::stringstream file_name;
                file_name << out_path.string() << "/" << cv_ptr->header.stamp << ".bmp"; // TODO: Could be "png", "tiff" or whatnot. The user could decide.
                cv::imwrite(file_name.str(), cv_ptr->image);
                // Show the image being saved
                cv::imshow(WINDOW, cv_ptr->image);
                cv::waitKey(3);
            }
            catch (cv_bridge::Exception& e)
            {
                std::cerr << "cv_bridge exception: " << e.what() << std::endl;
                return(-1);
            }
        }
    }

    bag.close();
    return(0);
}
