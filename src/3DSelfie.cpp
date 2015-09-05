// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

#include <iostream>
#include <mutex>
#include <vector>
#include <algorithm>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::cout;
using std::endl;


static pcl::OpenNIGrabber* kinectGrabber;
static boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
static std::mutex _mtx;  

std::vector<pcl::PointXYZRGBA>
find_3D_point(const std::vector<cv::Point> &pts, const pcl::PointCloud<pcl::PointXYZRGBA> &pc)
{   
    std::vector<pcl::PointXYZRGBA> out;

    for(auto pt2d = pts.begin() ; pt2d != pts.end(); ++pt2d)
    {
        for(auto pt3d = pc.begin() ; pt3d != pc.end(); ++pt3d)
        {
            // if(pt2d->x == pt3D->
        }
    }
    return out;
}

//Called every time there's a new image
void
grabber_callback_img(const openni_wrapper::Image::ConstPtr& img)
{   
    static bool first = true;
    static unsigned w;
    static unsigned h;
    static cv::Mat mat, mat2;

    if (first)
    {
        w = img->getWidth();
        h = img->getHeight();
        mat = cv::Mat(h, w, CV_8UC3);
       first = false; 
    }     

    //Convert to RGB
    img->fillRGB(w, h, mat.data);
    cv::cvtColor(mat, mat2, CV_RGB2BGR);

    //Display
    cv::imshow( "Image Viewer", mat2);
}

//Called every time there's a new point cloudframe 
void
grabber_callback_pc(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{        
    static bool first = true;

    if(first)
    {

        _mtx.lock();
        _viewer->addPointCloud<pcl::PointXYZRGBA>(cloud);
        _mtx.unlock();
        first = false;
    }
    else
    {
        _mtx.lock();
        _viewer->updatePointCloud(cloud);
        _mtx.unlock();
    }
}


//Creates and configures the 3D viewer, returning a shared pointer to it
void
create_3D_viewer()
{
    _viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // Set background color
    _viewer->setBackgroundColor (0.0, 0.0, 0.0);

    // Set camera position and orientation
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();
    _viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0); 
    _viewer->setSize (848, 480);
} 

void
create_img_viewer()
{
    cv::namedWindow("Image Viewer", 1);
}

int
main(int argc, char** argv)
{

    //Set up viewers
    create_3D_viewer();
    create_img_viewer();
    
    //Set up the grabber and register the callback function
    kinectGrabber = new pcl::OpenNIGrabber();
    if (kinectGrabber == 0) return false;
    
    //Register callbacks
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f_pc = boost::bind(&grabber_callback_pc, _1);
    boost::function<pcl::OpenNIGrabber::sig_cb_openni_image> f_img = 
        boost::bind(&grabber_callback_img, _1);
    
    kinectGrabber->registerCallback(f_pc);
    kinectGrabber->registerCallback(f_img);

    //Start the loop
    kinectGrabber->start();
    while (!_viewer->wasStopped ())
    {
        _mtx.lock();
        _viewer->spinOnce (100);
        _mtx.unlock();
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
        
    kinectGrabber->stop();
}
