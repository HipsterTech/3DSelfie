#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>

#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

#include <boost/program_options.hpp>

#include <vtkRenderWindow.h>

//Setting up a point cloud type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//Point clouds
static PointCloudT::Ptr cld_in(new PointCloudT), 
    cld_org(new PointCloudT), cld_icp(new PointCloudT);

//Visualization window
// static pcl::visualization::PCLVisualizer viewer("HipsterTech ICP Check");
static pcl::visualization::PCLVisualizer *viewer;
static int v1 (0), v2 (1);
static vtkSmartPointer< vtkRenderWindow > win;

//Icp object
static pcl::IterativeClosestPoint<PointT, PointT> icp;
static double max_dist;
static unsigned int period;

//Mutex to handle access to the point cloud inside the viewer.
static std::mutex _mtx;

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
    // Create window
    viewer = new pcl::visualization::PCLVisualizer("HipsterTech ICP Check");

    // Create two verticaly separated viewports
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // Set background color
    viewer->setBackgroundColor (1.0, 1.0, 1.0, v1);
    viewer->setBackgroundColor (1.0, 1.0, 1.0, v2);

    // Color Handling
    viewer->addPointCloud (cld_in, "cld_in_v1", v1);
    viewer->addPointCloud (cld_in, "cld_in_v2", v2);

    if(!check_color(*cld_org))
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> 
            cld_org_color_h (cld_org, 20, 180, 20); 
        viewer->addPointCloud (cld_org, cld_org_color_h, "cld_org_v1", v1);
    } 
    else
        viewer->addPointCloud (cld_org, "cld_org_v1", v1);

    if(!check_color(*cld_icp))
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> 
            cld_icp_color_h (cld_icp, 180, 20, 20);
        viewer->addPointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2", v2);
    } 
    else
        viewer->addPointCloud (cld_icp, "cld_icp_v2", v2);
    
     // Set camera position and orientation
    viewer->initCameraParameters ();
    viewer->setSize (1280, 720);
    viewer->setCameraPosition (0.0, -4.0, 0, 
        0.0, 0.0, 0.0,  0.0, 0.0, 1.0,  v1);
    viewer->setCameraPosition (0.0, -4.0, 0, 
        0.0, 0.0, 0.0,  0.0, 0.0, 1.0,  v2);
}

//Set up the properties of ICP
void
icp_setup()
{
    icp.setMaximumIterations (1);
    icp.setMaxCorrespondenceDistance(max_dist);
    icp.setInputSource (cld_icp);
    icp.setInputTarget (cld_in);

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
    _mtx.lock();
    viewer->updatePointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2");
    _mtx.unlock();
}

//Align parallel thread
void
compute_align(const bool color)
{
    pcl::console::TicToc time;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cld_icp_color_h (cld_icp, 180, 20, 20); 
    double delta_time = 0;
    boost::posix_time::ptime period_start, period_end;

    period_start = boost::posix_time::microsec_clock::local_time ();

    while(!viewer->wasStopped() && win->IsDrawable())
    {

        time.tic();
        icp.align (*cld_icp);
    
        if (icp.hasConverged())
        {    
            _mtx.lock();

            if(color)
                viewer->updatePointCloud (cld_icp, "cld_icp_v2");    
            else
                viewer->updatePointCloud (cld_icp, cld_icp_color_h, "cld_icp_v2");    

            _mtx.unlock();
            delta_time = time.toc();
        }

        cout << "Updated in: " << delta_time << " ms. Converged: " 
            << icp.hasConverged() << ". Error: " 
            << icp.getFitnessScore() << endl;

        period_end = boost::posix_time::microsec_clock::local_time ();
        if (period && 
                (static_cast<double>((period_end - period_start).total_seconds())) 
                    >= period)
        {
            reset_alignment();
            period_start = period_end;
            cout << "Resetting alignment." << endl;
        }
    }
}

int
parse_input_files(const boost::program_options::variables_map &vm)
{
    pcl::console::TicToc time;
    std::vector<int> nan_idx;
    std::vector<std::string> files;
    
    //No file inputs
    if(!vm.count("input-file"))
    {
        cout << "Usage: ht_icp_check file1 [file2] [options]" << endl;
        return -1;
    }
        

    files = vm["input-file"].as< std::vector<std::string> >();

    //At least one file was specified
    time.tic();
    if(pcl::io::load<PointT>(files[0], *cld_in) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", files[0].c_str());
        return -1;
    }
    pcl::removeNaNFromPointCloud(*cld_in, *cld_in, nan_idx);
    std::cout << "Loaded file " << files[0] << " (" << cld_in->size() 
        << " points) in " << time.toc() << " ms" << std::endl;

    //Deal with optional second file
    if(files.size() < 2)
    {
        *cld_org = *cld_in;
        return 0;
    }

    time.tic();
    if(pcl::io::load<PointT>(files[1], *cld_org) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", files[1].c_str());
        return -1;
    }
    pcl::removeNaNFromPointCloud(*cld_org, *cld_org, nan_idx);
    std::cout << "Loaded file " << files[1] << " (" << cld_org->size() 
    << " points) in " << time.toc() << " ms" << std::endl;

    return 0;
}

int
parse_transformations(const boost::program_options::variables_map &vm)
{
    bool trans(false), rot(false);
    std::vector<float> vals;
    Eigen::Matrix4d transformation_matrix(Eigen::Matrix4d::Identity());

    //Test for valid transation
    if (!vm["translate"].empty() && 
            (vals = vm["translate"].as< std::vector<float> >()).size() == 3) {
        transformation_matrix (0, 3) = vals[0];
        transformation_matrix (1, 3) = vals[1];
        transformation_matrix (2, 3) = vals[2];
        trans = true;
        cout << "t = [ " << vals[0] << " , " << vals[1] << " , "
            << vals[2] << " ]" << endl;
    }

    //Test for valid transation
    if (!vm["rotate"].empty() && 
            (vals = vm["rotate"].as< std::vector<float> >()).size() == 4) {
        Eigen::Vector3d k(vals[0], vals[1], vals[2]);
        Eigen::Matrix3d K, R;
        double angle = vals[3] * M_PI / 180.0;
        
        k.normalize();
        K << 0,     -k(2),  k(1), 
            k(2),   0,      -k(0),
            -k(1),  k(0),   0;

        R = Eigen::Matrix3d::Identity() + std::sin(angle) * K 
            + (1 - std::cos(angle)) * K * K;
        transformation_matrix.topLeftCorner<3,3>() = R;
        rot = true;
        cout << "R = " << endl << R << endl;
    }

    // Apply transformation if needed
    if(trans || rot)
        pcl::transformPointCloud (*cld_org, *cld_org, transformation_matrix);

    return 0;
}

int
parse_console_arguments(const int argc, const char **argv)
{
    boost::program_options::options_description desc("Allowed Options");
    boost::program_options::variables_map vm;
    boost::program_options::positional_options_description p;


    //Compose allowed options list
    desc.add_options()
        ("help", "produce help message")
        ("input-file", boost::program_options::value< std::vector<std::string> >(), 
            "input file(s) with point cloud")
        ("loop", boost::program_options::value<unsigned int>(&period)->default_value(0), 
            "set loop period t in s. If t equals 0, no loop takes place.")
        ("max-dist", boost::program_options::value<double>(&max_dist)->
            default_value(std::sqrt(std::numeric_limits<double>::max())), 
            "maximum correspondence distance")
        ("rotate", boost::program_options::value<std::vector<float> >()->multitoken(), 
            "rotation to be applied to second point cloud before icp. Rotation of angle "
            "(degrees) over vector v. --rotate vx vy vz angled")
        ("translate", boost::program_options::value<std::vector<float> >()->multitoken(), 
            "translation to be applied to second point cloud before icp. --translate tx ty tz")
        ;
    //Set positional options
    p.add("input-file", -1);

    try
    {
        //Parse commmand line and map values
        boost::program_options::store(
            boost::program_options::command_line_parser(argc, argv).options(desc)
            .style(boost::program_options::command_line_style::unix_style ^
                boost::program_options::command_line_style::allow_short)
            .positional(p).run(), vm);
        boost::program_options::notify(vm);
    }
    catch(const boost::program_options::error &ex)
    {
        cout << ex.what() << endl;
        return -1;
    }

    //print help description
    if (vm.count("help")) {
        cout << "Usage: ht_icp_check file1 [file2] [options]" << endl;
        cout << desc << "\n";
        return -1;
    }

    //parse loop description
    if (period)
        cout << "Loop period set to " << period << " s" << endl;

    //parse max correspondece dist
    if(max_dist < std::sqrt(std::numeric_limits<double>::max()))
        cout << "Maximum Correspondence Distance set to " << max_dist << endl;

    //parse file inputs
    if(parse_input_files(vm))
        return -1;

    //parse tranformations
    if(parse_transformations(vm))
        return -1;

    return 0;
}

int
main(int argc, char const *argv[])
{
    pcl::console::TicToc time;
	std::vector<int> nan_idx;

	/* Parse arguments */
    if(parse_console_arguments(argc, argv))
        return -1;

    //Initialize the icp cloud
    *cld_icp = *cld_org;  
	
	/* Visualization */
	visualization_setup();

    /* ICP Setup */
    icp_setup();

    /* VTK hook */
    win = viewer->getRenderWindow();  

    /* Spawn computational thread */
    std::thread align_thread(compute_align, check_color(*cld_icp));

	/* Window loop */
	while (!viewer->wasStopped() && win->IsDrawable())
	{
        _mtx.lock();
        viewer->spinOnce ();
        _mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

    align_thread.join();
    delete viewer;
	return 0;
}