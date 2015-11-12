#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

//Useful typedefs
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//Filthy globals
static PointCloudT::Ptr cld(new PointCloudT);
static PointCloudT::Ptr cld_r(new PointCloudT);
static pcl::visualization::PCLVisualizer::Ptr viewer;
static int v1 (0), v2 (1);

//Parameters
static float vx(0.01f), vy(0.01f), vz(0.01f);

/*
* void
* visualize()
*
* Configures and sets up our viewer window with 2 viewports.
*/
void
visualize()
{
	//Create objects
	viewer.reset(new pcl::visualization::PCLVisualizer("HipsterTech Voxel Grid Check"));

	// Create two verticaly separated viewports
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // Set background color
    viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor (1.0, 1.0, 1.0, v2);

    // Color Handling
    viewer->addPointCloud (cld, "cld_in_v1", v1);
    viewer->addPointCloud (cld_r, "cld_in_v2", v2);

    // Set camera position and orientation
    viewer->initCameraParameters ();
    viewer->setSize (1280, 720);
    viewer->setCameraPosition (0.0, -4.0, 0, 0.0, 0.0, 0.0,  0.0, 0.0, 1.0,  v1);
    viewer->setCameraPosition (0.0, -4.0, 0, 0.0, 0.0, 0.0,  0.0, 0.0, 1.0,  v2);
}

/*
* void
* voxel_filtering()
*
* Applies voxes filtering to out point cloud. 
*/
void
voxel_filtering()
{
	pcl::console::TicToc time;
	pcl::VoxelGrid<PointT> vox_grid;

	vox_grid.setInputCloud (cld);
	vox_grid.setLeafSize (vx, vy, vz);
	cout << "Starting filter now" << endl;
	time.tic();
	vox_grid.filter (*cld_r);
	cout << "Took " << time.toc()/1000.f 
		<< "(s) to filter cloud" << std::endl;
}

/*
* int
* parse_console_arguments(const int argc, char** const argv)
*
* Handles the parsing of all command line arguments. Returns
* non-zero if something is wrong.
*/
int
parse_console_arguments(const int argc, char** const argv)
{
	//Not enough arguments
	if(argc < 2)
		return -1;

	//Print help and usage
	if(pcl::console::find_switch(argc, argv, "-h")
		|| pcl::console::find_switch(argc, argv, "--help"))
	{
		cout << "Usage: ht_voxel_grid <file> [options]" << endl
			<< "Options" << endl
			<< "-h,--help\t prints this help message" << endl
			<< "-l lx,ly,lz \t set the leaf size for voxel downsampling" << endl;
		return 1;
	}

	//Load cloud
	if(pcl::io::load<PointT>(argv[1], *cld) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return -1;
    }

    //Parse leaf size
    pcl::console::parse_3x_arguments(argc, argv, "-l", vx, vy, vz);

	return 0;
}


int
main(int argc, char **argv)
{
	int ret(0);

	/* Parse arguments */
    if((ret = parse_console_arguments(argc, argv)))
        return ret;

    /* Voxel Filtering */
	voxel_filtering();

    /* Visualize result */
    visualize();

    /* VTK hook */
    vtkSmartPointer< vtkRenderWindow > win(viewer->getRenderWindow());	

    /* Main Loop */
	while (!viewer->wasStopped() && win->IsDrawable())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;
}