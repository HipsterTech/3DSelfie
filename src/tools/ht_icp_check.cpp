#include <iostream>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

//Setting up a point cloud type
typedef pcl::PointXYZRGBA PointT;
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

/*
*   check_color(const PointCloudT &cloud)
*   Checks if the cloud color information. The function that if the entire
*   cloud is black with alpha set to 255, then there's no color info.
*   Returns true if there is color info, false otherwise
*   
*   To unpack each component:
*   cout << "R: " << ((p->rgba >> 16) & 0xff) 
        << " G: " << ((p->rgba >> 8) & 0xff) 
        << " B: " << ((p->rgba) & 0xff) 
        << " A: " << ((p->rgba >> 24) & 0xff) << endl;
*/
bool
check_color(const PointCloudT &cloud)
{
    uint32_t color;

    if(cloud.empty())
        return false;

    color =  ((uint32_t)255 << 24);
    for (auto p = cloud.points.begin(); p != cloud.points.end(); ++p)
        if(p->rgba != color)
            return true;    
        
    return false;
}

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

    // Color Handling
    viewer.addPointCloud (cld_in, "cld_in_v1", v1);
    viewer.addPointCloud (cld_in, "cld_in_v2", v2);

    if(!check_color(*cld_org))
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> 
            cld_org_color_h (cld_org, 20, 180, 20); 
        viewer.addPointCloud (cld_org, cld_org_color_h, "cld_org_v1", v1);
    } 
    else
        viewer.addPointCloud (cld_org, "cld_org_v1", v1);

    if(!check_color(*cld_icp))
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> 
            cld_icp_color_h (cld_icp, 180, 20, 20);
        viewer.addPointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2", v2);
    } 
    else
        viewer.addPointCloud (cld_icp, "cld_icp_v2", v2);
    
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
    bool cld_has_color;
	

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
        // pcl::transformPointCloud (*cld_in, *cld_org, transformation_matrix);
    }

    //Initialize the icp cloud
    *cld_icp = *cld_org;  
	
	/* Visualization */
	visualization_setup();

    /* ICP Setup */
    icp_setup();

    /* Star the clock */
    time.tic();

    /* Check if cloud has color */
    cld_has_color = check_color(*cld_icp);

	/* Window loop */
	while (!viewer.wasStopped ())
	{
        // cout << "Rendering" << endl;
        if (icp.hasConverged())
        {
            if(cld_has_color)
            {
                viewer.updatePointCloud (cld_icp, "cld_icp_v2");    
            }
            else
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointT> 
                cld_icp_color_h (cld_icp, 180, 20, 20); 
                viewer.updatePointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2");    
            }
            cout << "Converged - Error: " << icp.getFitnessScore() << endl;
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