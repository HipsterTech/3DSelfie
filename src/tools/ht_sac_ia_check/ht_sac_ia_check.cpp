#include <pcl/console/time.h>
#include <pcl/io/auto_io.h>

#include <boost/program_options.hpp>

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
    if(files.size() < 1)
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
    int ret;

	/* Parse arguments */
    if((ret = parse_console_arguments(argc, argv))
        return ret;

    return 0;
}