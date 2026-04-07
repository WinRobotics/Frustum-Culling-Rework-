#include <iostream>

#include <pcl/filters/frustum_culling.h>
#include <pcl/point_types.h>        // PointXYZ, PointXYZRGB, etc.
#include <pcl/point_cloud.h>        // PointCloud
#include <pcl/io/pcd_io.h>          // loadPCDFile, savePCDFile
#include <pcl/filters/filter.h>     // removeNaNFromPointCloud
#include <pcl/visualization/pcl_visualizer.h> //using PCL viewer custom 

#include <X11/Xlib.h>  // add this to your includes

#include <string>
#include <thread>
#include <sstream>
#include <iostream>
#include <filesystem>


//./your_executable_name ../pcds
/***************************************************************************************************************************************************** */

using namespace std::chrono_literals;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> Pointcloud; 

struct PCD
{
    Pointcloud::Ptr cloud;
    std::string f_name;

    PCD(): cloud(std::make_shared<Pointcloud>()){}
};

/* Function to load PCD (Point Cloduds) from a directory. 
  param files_in_directory: path to directory containing PCD files
  param &data: vector with the loaded PCDs
*/

// void loadData (std::vector<std::filesystem::path> files_in_directory, std::vector<PCD, Eigen::aligned_allocator<PCD> > &data)
// {
//   PCL_INFO ("\n\nLoading PCD files...\n\n");
//   // Go over all of the entries in the path
//   for (const auto & entry : files_in_directory){
//     // if the entry is a file with extension .pcd, load it
//     if (entry.extension() == ".pcd"){
      
//       // Create pcd structure, assign it the path of the file as name and load the pcd to the cloud portion
//       PCD p;
//       p.f_name = entry.string();
//       pcl::io::loadPCDFile (entry.string(), *p.cloud);

//       // Remove NAN points from the cloud
//       std::vector<int> indices;
//       pcl::removeNaNFromPointCloud(*p.cloud,*p.cloud, indices);

//       // Add PCD structure to the vector
//       data.push_back (p);
//     }
//   }
    
  
// }


void filter(std::vector<PCD,Eigen::aligned_allocator<PCD>> & data)
{
  PCL_INFO ("\n\nStaring to apply filter\n\n");
  Pointcloud::Ptr cloud_filtered(new Pointcloud);

  //cretae the filter
  pcl::FrustumCulling<PointT> fc;
  //The following parameters were defined by trial and error

  fc.setVerticalFOV(100.0);
  fc.setHorizontalFOV(100.0);
  fc.setNearPlaneDistance(0.0);
  fc.setFarPlaneDistance(150.0);

  //Define the camera pose
  Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f rotation = Eigen::Quaternionf(0.9968173,0,0,0.0784951).toRotationMatrix();
  /*
   PCL is mostly column-vector like the rest of the Eigen ecosystem. 
   FrustumCulling is one of the weird exceptions because it inherited its camera model from VTK.
   */
  Eigen::RowVector3f translation(0,0,0);
  //This indicate the direction which to look in the point cloud
  //Eigen::RowVector3f — the translation is explicitly declared as a row vector, not Eigen::Vector3f (which would be a column vector)
  /*So block(3, 0, 1, 3) means:

      Start at row 3
      Start at col 0
      Take 1 row tall
      Take 3 cols wide
      It's not "row 3, col 0, cols 1–3" — the last two arguments are the size of the block, not indices. So it selects this slice:

      row 3, col 0  |  row 3, col 1  |  row 3, col 2
          tx                ty               tz
      Same idea for block(0, 0, 3, 3) — start at row 0 col 0, grab a 3×3 chunk → the rotation matrix.
  */
  camera_pose.block(0,0,3,3) = rotation;
  camera_pose.block(3,0,1,3) = translation;
  cout<<"Camera Pose "<<endl<<camera_pose<<endl<<endl;
  /*
  [ r00  r01  r02  0 ]
  [ r10  r11  r12  0 ]
  [ r20  r21  r22  0 ]
  [  tx   ty   tz  1 ]
  */
  fc.setCameraPose (camera_pose);

  for(auto& points : data)
  {

    Pointcloud::Ptr cloud_filtered(new Pointcloud);
    fc.setInputCloud(points.cloud);
    fc.filter(*cloud_filtered);
    //Update the cloud
    points.cloud = cloud_filtered;
    //Replace the PCD file with the filtered data
    pcl::io::savePCDFileASCII(points.f_name,*points.cloud);


  }
  


      
}


void loadData (std::vector<std::filesystem::path> files_in_directory,std::vector<PCD,Eigen::aligned_allocator<PCD> >&data)
{
  PCL_INFO ("\n\nLoading PCD files...\n\n");
  // Go over all of the entries in the path
  for(const auto &entry : files_in_directory)
  {
        if(entry.extension() == ".pcd")
        {
            //load the pcd file, assign a PCD struct, load it into memory of cloud
            PCD p;
            p.f_name = entry.string();
            pcl::io::loadPCDFile(entry.string(),*p.cloud);

            //Remove NAN points from cloud
            std::vector<int> indicies;
            pcl::removeNaNFromPointCloud(*p.cloud,*p.cloud,indicies);

            //Add PCD structure to the vecot
            data.push_back(p);

        }
  }

}

// TODO(sherwin): add more coordinate PCL frame to understand how the transformation was done 
pcl::visualization::PCLVisualizer::Ptr simpleVis (Pointcloud::ConstPtr cloud)
{
  //open 3D Viewer and add point cloud
  pcl::visualization::PCLVisualizer::Ptr viewer (std::make_shared<pcl::visualization::PCLVisualizer>("Custom 3D Viewer"));
  viewer->setBackgroundColor(0,0,0);
  viewer->addPointCloud<PointT> (cloud,"sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}




int main(int argc, char** argv)
{
     // XInitThreads();  // must be first call
      if (argc < 2){
      PCL_ERROR ("Error: Syntax is: %s <path_to_pcds>\n\n", argv[0]);
      return -1;
      }
      //Get the PCD path using the directory path received as argument then sort them
      std::vector<std::filesystem::path> files_in_directory;
      std::copy(
        std::filesystem::directory_iterator(argv[1]),   // start: first entry in directory
        std::filesystem::directory_iterator(),          // end: default-constructed = "past the last entry"
        std::back_inserter(files_in_directory)          // where to put them: push_back into the vector
      );
      std::sort(files_in_directory.begin(), files_in_directory.end());

      //load data
      std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
      loadData(files_in_directory,data);

      // Check user input
      if (data.empty ())
      {
        PCL_ERROR ("Syntax is: %s path_to_pcds ", argv[0]);
        
        return (-1);
      }
      PCL_INFO ("Loaded %d Point Clouds.", (int)data.size ());
      PCL_INFO ("Cloud 0 has %d points.", (int)data[0].cloud->size());

      filter(data);

      pcl::visualization::PCLVisualizer::Ptr viewer;
      viewer = simpleVis(data[0].cloud);

      // pcl::visualization::PCLVisualizer::Ptr viewer(std::make_shared<pcl::visualization::PCLVisualizer>("Test Viewer"));
      // viewer->setBackgroundColor(0,0,0);
      // viewer->addCoordinateSystem(1.0);
      // viewer->initCameraParameters();


      viewer->spin();  // blocks until window is closed
      // while (!viewer->wasStopped())
      // {
      //   viewer->spinOnce(100);
      //   std::this_thread::sleep_for(100ms);
      // }

      //filter(data);

      // viewer->spin();
      // viewer = simpleVis(data[0].cloud);
            

}