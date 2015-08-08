// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

#include <iostream>
#include <mutex>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;


static pcl::Grabber* kinectGrabber;
static boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
static std::mutex _mtx;  
std::vector<pcl::visualization::Camera> cam; 

//Called every time there's a new frame 
void
grabber_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{        
    static bool first = true;

    if(first)
    {
        _mtx.lock();
        _viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "samplecloud");
        _mtx.unlock();
        //_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        first = false;
        return;
    }
    else
    {
        _mtx.lock();
        _viewer->updatePointCloud(cloud,"samplecloud");
        _viewer->getCameras(cam); 
        _mtx.unlock();    
    }

    cout << "Cam: " << endl 
             << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
             << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
             << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;
     

}

//Creates and configures the viewer, returning a shared pointer to it
boost::shared_ptr<pcl::visualization::PCLVisualizer>
create_viewer()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0,0,-1,0,-1,0,0,0,1); 
    viewer->getCameras(cam); 
    return (viewer);    
} 


int
main(int argc, char** argv)
{

    //Set up viewer
    _viewer = create_viewer();
    
    //Set up the grabber and register the callback function
    kinectGrabber = new pcl::OpenNIGrabber();
    if (kinectGrabber == 0) return false;
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&grabber_callback, _1);
    kinectGrabber->registerCallback(f);

    //Start the loop
    kinectGrabber->start();
    while (!_viewer->wasStopped ())
    {
        _mtx.lock();
        _viewer->spinOnce (100);
        _mtx.unlock();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
        
    kinectGrabber->stop();
}
