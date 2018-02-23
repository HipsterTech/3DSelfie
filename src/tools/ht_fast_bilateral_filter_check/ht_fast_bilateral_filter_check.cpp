#include <pcl/console/time.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/io/auto_io.h>

//Useful typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//Filthy globals
static PointCloudT::Ptr cld(new PointCloudT);
static PointCloudT::Ptr cld_f(new PointCloudT);

int 
fast_bilateral_filter()
{
	pcl::console::TicToc time;
	pcl::FastBilateralFilter<PointT> filter;
	
	filter.setInputCloud(cld);
	filter.setSigmaS(5);
	filter.setSigmaR(5e-3);
	time.tic();
	filter.applyFilter(*cld_f);
	std::cout << "Took " << time.toc()/1000.f 
		<< "(s) to apply fast bilateral filter" << std::endl;
	return 0;
}

int
parse_console_arguments(const int argc, char** const argv)
{
	//Not enough arguments
	if(argc < 2)
		return -1;

	//Load cloud
	if(pcl::io::load<PointT>(argv[1], *cld) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return -1;
    }

	return 0;
}


int
main(int argc, char **argv)
{
	int ret(0);

	/* Parse arguments */
    if((ret = parse_console_arguments(argc, argv)))
        return ret;

    /* Apply fast bilateral filtering */
    fast_bilateral_filter();

return 0;
}