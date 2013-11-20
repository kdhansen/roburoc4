/*  
   bag_to_pcd
    A tool for saving pointclouds from a topic in a .bag file
    
    By Karl D. Hansen
    Aalborg University
    19-10-2012
*/

#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
        std::cerr << "Example: " << program_path.filename().string() << " data.bag /camera/cloud_raw ./PCDs" << std::endl;
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
        sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
        
        if (i != NULL)
        {
            // Save the cloud
            std::stringstream file_name;
            file_name << out_path.string() << "/" << i->header.stamp << ".pcd";
            pcl::io::savePCDFile(file_name.str(), *i, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);
        }
    }

    bag.close();
    return(0);
}
