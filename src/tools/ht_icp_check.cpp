#include <iostream>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

//Setting up a point cloud type
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//Point clouds
static PointCloudT::Ptr cld_in(new PointCloudT), 
    cld_org(new PointCloudT), cld_icp(new PointCloudT);

//Visualization window
static pcl::visualization::PCLVisualizer viewer("HipsterTech ICP Check");
static int v1 (0), v2 (1);

//Icp object
static pcl::IterativeClosestPoint<PointT, PointT> icp;
static unsigned int period = 4000;



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

    // Set the point cloud color handlers
    pcl::visualization::PointCloudColorHandlerCustom<PointT> 
        cld_in_color_h (cld_in, 0, 0,0); 
    pcl::visualization::PointCloudColorHandlerCustom<PointT> 
        cld_org_color_h (cld_org, 20, 180, 20); 
    pcl::visualization::PointCloudColorHandlerCustom<PointT> 
        cld_icp_color_h (cld_icp, 180, 20, 20);  

    /* Add point clouds to viewports */
    viewer.addPointCloud (cld_in, cld_in_color_h, "cld_in_v1", v1);
    viewer.addPointCloud (cld_org, cld_org_color_h, "cld_org_v1", v1);
    viewer.addPointCloud (cld_in, cld_in_color_h, "cld_in_v2", v2);
    viewer.addPointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2", v2);

     // Set camera position and orientation
    viewer.initCameraParameters ();
    viewer.setSize (1280, 720);
    viewer.setCameraPosition (0.0, -4.0, 0, 
        0.0, 0.0, 0.0,  0.0, 0.0, 1.0,  v1);
    viewer.setCameraPosition (0.0, -4.0, 0, 
        0.0, 0.0, 0.0,  0.0, 0.0, 1.0,  v2);
}

//Set up the properties of ICP
void
icp_setup()
{
    icp.setMaximumIterations (1);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setInputSource (cld_icp);
    icp.setInputTarget (cld_in);
    icp.align (*cld_icp);

    //Attempting to set some RANSAC thing
    // icp.setRANSACIterations(100000);
    // icp.setRANSACOutlierRejectionThreshold(0.001);
}

/*
*   reset_alignment()
*   Invoked periodically to visually the loop again. 
*/
void
reset_alignment()
{
    *cld_icp = *cld_org;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> 
                cld_icp_color_h (cld_icp, 180, 20, 20); 
            cout << "Converged - Error: " << icp.getFitnessScore() << endl;
    viewer.updatePointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2");
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

    /* Test for 2nd possible file */
    if(argc > 2)
    {
        time.tic();
        if(pcl::io::load<PointT>(argv[2], *cld_org) < 0)
        {
            PCL_ERROR ("Error loading cloud %s.\n", argv[2]);
            return -1;
        }
        std::cout << "Loaded file " << argv[2] << " (" << cld_org->size() 
        << " points) in " << time.toc() << " ms" << std::endl;

        // Executing the transformation
        pcl::transformPointCloud (*cld_org, *cld_org, transformation_matrix);
    }
    else
    {
        // Display in terminal the transformation matrix
        std::cout << "Applying rigid transformation to: cld_in -> cld_icp" << std::endl;

        // Executing the transformation
        pcl::transformPointCloud (*cld_in, *cld_org, transformation_matrix);
    }

    //Initialize the icp cloud
    *cld_icp = *cld_org;  
	
	/* Visualization */
	visualization_setup();

    /* ICP Setup */
    icp_setup();

    /* Star the clock */
    time.tic();

	/* Window loop */
	while (!viewer.wasStopped ())
	{
        // cout << "Rendering" << endl;
        if (icp.hasConverged())
        {
            pcl::visualization::PointCloudColorHandlerCustom<PointT> 
                cld_icp_color_h (cld_icp, 180, 20, 20); 
            cout << "Converged - Error: " << icp.getFitnessScore() << endl;
            viewer.updatePointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2");
            icp.align (*cld_icp);
        }

        if (time.toc() >= period * 1000)
        {
            reset_alignment();
            time.tic();
        }
        viewer.spinOnce ();
	}

	return 0;
}