#include <iostream>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

//Setting up a point cloud type
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//Globals ^^ 
static PointCloudT::Ptr cld_in(new PointCloudT), 
    cld_tgt(new PointCloudT), cld_icp(new PointCloudT);//
static pcl::visualization::PCLVisualizer viewer("HipsterTech ICP Check");
static int v1 (0), v2 (1);

//Sets up everything related to visualization
void
visualization_setup()
{
    // Create two verticaly separated viewports
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // Set background color
    viewer.setBackgroundColor (1.0, 1.0, 1.0, v1);
    viewer.setBackgroundColor (1.0, 1.0, 1.0, v2);

    // Set the point cloud to black
    pcl::visualization::PointCloudColorHandlerCustom<PointT> 
        cld_in_color_h (cld_in, 0, 0,0); 
    pcl::visualization::PointCloudColorHandlerCustom<PointT> 
        cld_tgt_color_h (cld_tgt, 20, 180, 20);   

    /* Add point clouds to viewports */
    viewer.addPointCloud (cld_in, cld_in_color_h, "cld_in_v1", v1);
    viewer.addPointCloud (cld_tgt, cld_tgt_color_h, "cld_tgt_v1", v1);

     // Set camera position and orientation
    viewer.setSize (1280, 1024);
    viewer.setCameraPosition (0.0, -10.0, 0, 
        0.0, 0.0, 0.0, v1);
    // viewer.setCameraPosition (0.0, -10.0, 0,  
    //     0.0, 0.0, 0.0, v2);
}

int
main(int argc, char const *argv[])
{
	pcl::console::TicToc time;
	

	/* Parse arguments */
	if (argc < 2)
	{
		std::cout << "Usage :" << std::endl
			<< argv[0] << " <file> [number_of_ICP_iterations]" 
			<< std::endl;
    	PCL_ERROR ("Provide one mesh file.\n");
    	return -1;
	}

	/* Load file */
	time.tic();
	if(pcl::io::load<PointT>(argv[1], *cld_in) < 0)
	{
		PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    	return -1;
	}
	std::cout << "Loaded file " << argv[1] << " (" << cld_in->size() 
		<< " points) in " << time.toc() << " ms" << std::endl;

    /* Test for 2nd possible file */
    if(argc > 2)
    {
        time.tic();
        if(pcl::io::load<PointT>(argv[2], *cld_tgt) < 0)
        {
            PCL_ERROR ("Error loading cloud %s.\n", argv[2]);
            return -1;
        }
        std::cout << "Loaded file " << argv[2] << " (" << cld_tgt->size() 
        << " points) in " << time.toc() << " ms" << std::endl;
    }
    else
    {
        // Defining a rotation matrix and translation vector
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

        // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        double theta = M_PI / 8;  // The angle of rotation in radians
        transformation_matrix (0, 0) = cos (theta);
        transformation_matrix (0, 1) = -sin (theta);
        transformation_matrix (1, 0) = sin (theta);
        transformation_matrix (1, 1) = cos (theta);

        // A translation on Z axis (0.4 meters)
        transformation_matrix (2, 3) = 0.4;

        // Display in terminal the transformation matrix
        std::cout << "Applying rigid transformation to: cld_in -> cld_icp" << std::endl;


        // Executing the transformation
        pcl::transformPointCloud (*cld_in, *cld_icp, transformation_matrix);
        *cld_tgt = *cld_icp;  // We backup cloud_icp into cloud_tr for later use
    }

	
	/* Visualization */
	visualization_setup();

	/* Window loop */
	while (!viewer.wasStopped ())
	{
        viewer.spinOnce ();
	}

	return 0;
}