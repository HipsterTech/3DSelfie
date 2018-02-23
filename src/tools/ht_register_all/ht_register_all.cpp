#include <vector>

// #include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>

//Useful typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Application data structures
static std::vector<PointCloudT::Ptr> pclouds_, pclouds_orig_;
static std::vector<pcl::PointCloud<pcl::Normal>::Ptr> pcloud_normals_;
static std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> pclouds_features_;

void
reserve_all (size_t n)
{
  pclouds_orig_.reserve (n);
  pclouds_.reserve (n);
  pcloud_normals_.reserve (n);
}

int
parse_file (const char *filename)
{
  PointCloudT::Ptr cld (new PointCloudT);

  // If problems error out
  if (pcl::io::load<PointT> (filename, *cld) < 0)
  {
      PCL_ERROR ("Error loading cloud %s.\n", filename);
      return -1;
  }

  // If not dense, filter NaNs
  if (!cld->is_dense)
  {
    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud (*cld, *cld, nan_idx);
  }

  std::cout << filename << std::endl; 
  pclouds_orig_.push_back (cld);
  pclouds_.push_back (PointCloudT::Ptr (new PointCloudT));
  return 0;
}

int
parse_console_arguments (const int argc, const char** argv)
{
  //Not enough arguments
  if (argc < 2)
    return -1;

  //Perform the necessary allocations
  reserve_all (argc - 1);

  //Parse all remaining arguments 
  std::cout << "Loading File(s)" << std::endl; 
  for (size_t i = 1; i < argc; ++i)
    if (parse_file (argv[i]))
      return -1;

    return 0;
}

void
voxel_decimate ()
{
  std::cout << "Starting Voxel Decimation" << std::endl;

  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
  for (size_t i = 0; i < pclouds_orig_.size (); ++i)
  {
    vox_grid.setInputCloud (pclouds_orig_[i]);  
    vox_grid.filter (*pclouds_[i]);  
    std::cout << "Pre: " <<  pclouds_orig_[i]->size ()
      << " Post: " << pclouds_[i]->size () 
      << " Red: " << static_cast<float> (pclouds_[i]->size ()) / 
      static_cast<float> (pclouds_orig_[i]->size ()) * 100.f << std::endl;    
  }
}

void
compute_normals ()
{
  std::cout << "Starting Normal Computation" << std::endl;

  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.04f);
  ne.setViewPoint (0.f, 0.f, 0.f);
  for (size_t i = 0; i < pclouds_.size (); ++i)
  {
    ne.setInputCloud (pclouds_[i]);
    pcloud_normals_.push_back (pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>));
    ne.compute(*pcloud_normals_[i]);
    std::cout << "Finished processing cloud " << i << " normals." << std::endl;
  }
}

void
compute_features ()
{
  std::cout << "Starting Feature Computation" << std::endl;

  pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // static std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> pclouds_features_;

  fpfh_est.setSearchMethod (tree);
  for (size_t i = 0; i < pclouds_.size (); ++i)
  {
    pclouds_features_.push_back (pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new pcl::PointCloud<pcl::FPFHSignature33>));
    fpfh_est.setInputCloud (pclouds_[i]);
    fpfh_est.setInputNormals (pcloud_normals_[i]);
    fpfh_est.setRadiusSearch (0.04f);
    fpfh_est.compute (*pclouds_features_[i]);  
    std::cout << "Finished processing cloud " << i << " features." << std::endl;
  } 
}

int
main (int argc, char const *argv[])
{
  int ret(0);

  /* Parse arguments */
  if ( (ret = parse_console_arguments (argc, argv)))
    return ret;

  /* VoXel decimate all */
  voxel_decimate ();

  /* Compute Normals */
  compute_normals ();

  /* Compute Local Features */
  compute_features ();

	/* code */
	return 0;
}