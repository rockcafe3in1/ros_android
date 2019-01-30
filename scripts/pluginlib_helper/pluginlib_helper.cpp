/**
 * This file was automatically generated.
 * Helper file for the static version of ROS pluginlib.
 * 
 * The function is called by the pluginlib classLoader constructor to
 * get the mapping between a library and it's exported plugin classes,
 * as defined in the plugin description xml file.
 * This is required in systems without access to the filesystem,
 * e.g. Android and the plugins are statically linked at build time.
 * 
 */

#include "boost/algorithm/string.hpp"
#include "class_loader/multi_library_class_loader.h"
#include <map>
#include <pluginlib/class_desc.h>

using namespace pluginlib;

typedef std::map<std::string, ClassDesc> classes_available_map;
typedef std::pair<std::string, ClassDesc> plugin_pair;
classes_available_map getStaticClassesAvailable(void)
{
  classes_available_map pluginClasses;
  pluginClasses.insert(
    plugin_pair("carrot_planner/CarrotPlanner",
      ClassDesc("carrot_planner/CarrotPlanner",
                "carrot_planner::CarrotPlanner",
                "nav_core::BaseGlobalPlanner",
                "carrot_planner",
                "A simple planner that seeks to place a legal carrot in-front of the robot",
                "lib/libcarrot_planner",
                "")));
  pluginClasses.insert(
    plugin_pair("rotate_recovery/RotateRecovery",
      ClassDesc("rotate_recovery/RotateRecovery",
                "rotate_recovery::RotateRecovery",
                "nav_core::RecoveryBehavior",
                "rotate_recovery",
                "A recovery behavior that performs a 360 degree in-place rotation to attempt to clear out space.",
                "lib/librotate_recovery",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MultiChannelMedianFilterDouble",
      ClassDesc("filters/MultiChannelMedianFilterDouble",
                "filters::MultiChannelMedianFilter<double>",
                "filters::MultiChannelFilterBase<double>",
                "filters",
                "This is a median filter which works on a stream of std::vector of doubles.",
                "lib/libmedian",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MedianFilterDouble",
      ClassDesc("filters/MedianFilterDouble",
                "filters::MedianFilter<double>",
                "filters::FilterBase<double>",
                "filters",
                "This is a median filter which works on a stream of doubles.",
                "lib/libmedian",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MultiChannelMedianFilterFloat",
      ClassDesc("filters/MultiChannelMedianFilterFloat",
                "filters::MultiChannelMedianFilter<float>",
                "filters::MultiChannelFilterBase<float>",
                "filters",
                "This is a median filter which works on a stream of std::vector of floats.",
                "lib/libmedian",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MedianFilterFloat",
      ClassDesc("filters/MedianFilterFloat",
                "filters::MedianFilter<float>",
                "filters::FilterBase<float>",
                "filters",
                "This is a median filter which works on a stream of floats.",
                "lib/libmedian",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MeanFilterDouble",
      ClassDesc("filters/MeanFilterDouble",
                "filters::MeanFilter<double>",
                "filters::FilterBase<double>",
                "filters",
                "This is a mean filter which works on a stream of doubles.",
                "lib/libmean",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MeanFilterFloat",
      ClassDesc("filters/MeanFilterFloat",
                "filters::MeanFilter<float>",
                "filters::FilterBase<float>",
                "filters",
                "This is a mean filter which works on a stream of floats.",
                "lib/libmean",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MultiChannelMeanFilterDouble",
      ClassDesc("filters/MultiChannelMeanFilterDouble",
                "filters::MultiChannelMeanFilter<double>",
                "filters::MultiChannelFilterBase<double>",
                "filters",
                "This is a mean filter which works on a stream of vectors of doubles.",
                "lib/libmean",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MultiChannelMeanFilterFloat",
      ClassDesc("filters/MultiChannelMeanFilterFloat",
                "filters::MultiChannelMeanFilter<float>",
                "filters::MultiChannelFilterBase<float>",
                "filters",
                "This is a mean filter which works on a stream of vectors of floats.",
                "lib/libmean",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/ParamTest",
      ClassDesc("filters/ParamTest",
                "filters::ParamTest<double>",
                "filters::FilterBase<double>",
                "filters",
                "This is a filter designed to test parameter readings. It's not useful.",
                "lib/libtest_param",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/IncrementFilterInt",
      ClassDesc("filters/IncrementFilterInt",
                "filters::IncrementFilter<int>",
                "filters::FilterBase<int>",
                "filters",
                "This is a increment filter which works on a stream of ints.",
                "lib/libincrement",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MultiChannelIncrementFilterInt",
      ClassDesc("filters/MultiChannelIncrementFilterInt",
                "filters::MultiChannelIncrementFilter<int>",
                "filters::MultiChannelFilterBase<int>",
                "filters",
                "This is a increment filter which works on a stream of vectors of ints.",
                "lib/libincrement",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/MultiChannelTransferFunctionFilterDouble",
      ClassDesc("filters/MultiChannelTransferFunctionFilterDouble",
                "filters::MultiChannelTransferFunctionFilter<double>",
                "filters::MultiChannelFilterBase<double>",
                "filters",
                "This is a transfer filter which works on a stream of doubles.",
                "lib/libtransfer_function",
                "")));
  pluginClasses.insert(
    plugin_pair("filters/TransferFunctionFilterDouble",
      ClassDesc("filters/TransferFunctionFilterDouble",
                "filters::SingleChannelTransferFunctionFilter<double>",
                "filters::FilterBase<double>",
                "filters",
                "This is a transfer filter which works on a stream of doubles.",
                "lib/libtransfer_function",
                "")));
  pluginClasses.insert(
    plugin_pair("navfn/NavfnROS",
      ClassDesc("navfn/NavfnROS",
                "navfn::NavfnROS",
                "nav_core::BaseGlobalPlanner",
                "navfn",
                "A implementation of a grid based planner using Dijkstra",
                "lib/libnavfn",
                "")));
  pluginClasses.insert(
    plugin_pair("dwa_local_planner/DWAPlannerROS",
      ClassDesc("dwa_local_planner/DWAPlannerROS",
                "dwa_local_planner::DWAPlannerROS",
                "nav_core::BaseLocalPlanner",
                "dwa_local_planner",
                "A implementation of a local planner using either a DWA approach based on configuration parameters.",
                "lib/libdwa_local_planner",
                "")));
  pluginClasses.insert(
    plugin_pair("costmap_2d::InflationLayer",
      ClassDesc("costmap_2d::InflationLayer",
                "costmap_2d::InflationLayer",
                "costmap_2d::Layer",
                "costmap_2d",
                "Inflates obstacles to speed collision checking and to make robot prefer to stay away from obstacles.",
                "liblayers",
                "")));
  pluginClasses.insert(
    plugin_pair("costmap_2d::ObstacleLayer",
      ClassDesc("costmap_2d::ObstacleLayer",
                "costmap_2d::ObstacleLayer",
                "costmap_2d::Layer",
                "costmap_2d",
                "Listens to laser scan and point cloud messages and marks and clears grid cells.",
                "liblayers",
                "")));
  pluginClasses.insert(
    plugin_pair("costmap_2d::StaticLayer",
      ClassDesc("costmap_2d::StaticLayer",
                "costmap_2d::StaticLayer",
                "costmap_2d::Layer",
                "costmap_2d",
                "Listens to OccupancyGrid messages and copies them in, like from map_server.",
                "liblayers",
                "")));
  pluginClasses.insert(
    plugin_pair("costmap_2d::VoxelLayer",
      ClassDesc("costmap_2d::VoxelLayer",
                "costmap_2d::VoxelLayer",
                "costmap_2d::Layer",
                "costmap_2d",
                "Similar to obstacle costmap, but uses 3D voxel grid to store data.",
                "liblayers",
                "")));
  pluginClasses.insert(
    plugin_pair("global_planner/GlobalPlanner",
      ClassDesc("global_planner/GlobalPlanner",
                "global_planner::GlobalPlanner",
                "nav_core::BaseGlobalPlanner",
                "global_planner",
                "A implementation of a grid based planner using Dijkstras or A*",
                "lib/libglobal_planner",
                "")));
  pluginClasses.insert(
    plugin_pair("clear_costmap_recovery/ClearCostmapRecovery",
      ClassDesc("clear_costmap_recovery/ClearCostmapRecovery",
                "clear_costmap_recovery::ClearCostmapRecovery",
                "nav_core::RecoveryBehavior",
                "clear_costmap_recovery",
                "A recovery behavior that reverts the costmap to the static map outside of a user specified window.",
                "lib/libclear_costmap_recovery",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/raw_pub",
      ClassDesc("image_transport/raw_pub",
                "image_transport::RawPublisher",
                "image_transport::PublisherPlugin",
                "image_transport",
                "This is the default publisher. It publishes the Image as-is on the base topic.",
                "lib/libimage_transport_plugins",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/raw_sub",
      ClassDesc("image_transport/raw_sub",
                "image_transport::RawSubscriber",
                "image_transport::SubscriberPlugin",
                "image_transport",
                "This is the default pass-through subscriber for topics of type sensor_msgs/Image.",
                "lib/libimage_transport_plugins",
                "")));
  pluginClasses.insert(
    plugin_pair("move_slow_and_clear/MoveSlowAndClear",
      ClassDesc("move_slow_and_clear/MoveSlowAndClear",
                "move_slow_and_clear::MoveSlowAndClear",
                "nav_core::RecoveryBehavior",
                "move_slow_and_clear",
                "A recovery behavior that clears information in the costmap within the circumscribed radius of the robot and then limits the speed of the robot. Note, this recovery behavior is not truly safe, the robot may hit things, it'll just happen at a user-specified speed. Also, this recovery behavior is only compatible with local planners that allow maximum speeds to be set via dynamic reconfigure.",
                "lib/libmove_slow_and_clear",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/BoundaryEstimation",
      ClassDesc("pcl/BoundaryEstimation",
                "BoundaryEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "BoundaryEstimation estimates whether a set of points is lying on surface boundaries using an angle criterion. The code makes use of the estimated surface normals at each point in the input data set.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/FPFHEstimation",
      ClassDesc("pcl/FPFHEstimation",
                "FPFHEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "FPFHEstimation estimates the Fast Point Feature Histogram (FPFH) descriptor for a given point cloud dataset containing points and normals.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/FPFHEstimationOMP",
      ClassDesc("pcl/FPFHEstimationOMP",
                "FPFHEstimationOMP",
                "nodelet::Nodelet",
                "pcl_ros",
                "FPFHEstimationOMP estimates the Fast Point Feature Histogram (FPFH) descriptor for a given point cloud dataset containing points and normals, in parallel, using the OpenMP standard.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/SHOTEstimation",
      ClassDesc("pcl/SHOTEstimation",
                "SHOTEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "SHOTEstimation estimates SHOT descriptor for a given point cloud dataset containing points and normals.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/SHOTEstimationOMP",
      ClassDesc("pcl/SHOTEstimationOMP",
                "SHOTEstimationOMP",
                "nodelet::Nodelet",
                "pcl_ros",
                "SHOTEstimationOMP estimates SHOT descriptor for a given point cloud dataset containing points and normals, in parallel, using the OpenMP standard.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/MomentInvariantsEstimation",
      ClassDesc("pcl/MomentInvariantsEstimation",
                "MomentInvariantsEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "MomentInvariantsEstimation estimates the 3 moment invariants (j1, j2, j3) at each 3D point.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/NormalEstimationOMP",
      ClassDesc("pcl/NormalEstimationOMP",
                "NormalEstimationOMP",
                "nodelet::Nodelet",
                "pcl_ros",
                "NormalEstimationOMP estimates local surface properties at each 3D point, such as surface normals and curvatures, in parallel, using the OpenMP standard.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/NormalEstimationTBB",
      ClassDesc("pcl/NormalEstimationTBB",
                "NormalEstimationTBB",
                "nodelet::Nodelet",
                "pcl_ros",
                "NormalEstimationTBB estimates local surface properties at each 3D point, such as surface normals and curvatures, in parallel, using Intel's Threading Building Blocks library.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/NormalEstimation",
      ClassDesc("pcl/NormalEstimation",
                "NormalEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "NormalEstimation estimates local surface properties at each 3D point, such as surface normals and curvatures.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PFHEstimation",
      ClassDesc("pcl/PFHEstimation",
                "PFHEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "PFHEstimation estimates the Point Feature Histogram (PFH) descriptor for a given point cloud dataset containing points and normals.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PrincipalCurvaturesEstimation",
      ClassDesc("pcl/PrincipalCurvaturesEstimation",
                "PrincipalCurvaturesEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "PrincipalCurvaturesEstimation estimates the directions (eigenvectors) and magnitudes (eigenvalues) of principal surface curvatures for a given point cloud dataset containing points and normals.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/VFHEstimation",
      ClassDesc("pcl/VFHEstimation",
                "VFHEstimation",
                "nodelet::Nodelet",
                "pcl_ros",
                "VFHEstimation estimates the Viewpoint Feature Histogram (VFH) global descriptor for a given point cloud cluster dataset containing points and normals.",
                "lib/libpcl_ros_features",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PassThrough",
      ClassDesc("pcl/PassThrough",
                "PassThrough",
                "nodelet::Nodelet",
                "pcl_ros",
                "PassThrough is a filter that uses the basic Filter class mechanisms for passing data around.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/VoxelGrid",
      ClassDesc("pcl/VoxelGrid",
                "VoxelGrid",
                "nodelet::Nodelet",
                "pcl_ros",
                "VoxelGrid assembles a local 3D grid over a given PointCloud, and uses that to downsample the data.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/ProjectInliers",
      ClassDesc("pcl/ProjectInliers",
                "ProjectInliers",
                "nodelet::Nodelet",
                "pcl_ros",
                "ProjectInliers uses a model and a set of inlier indices from a PointCloud to project them into a separate PointCloud.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/ExtractIndices",
      ClassDesc("pcl/ExtractIndices",
                "ExtractIndices",
                "nodelet::Nodelet",
                "pcl_ros",
                "ExtractIndices extracts a set of indices from a PointCloud as a separate PointCloud.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/StatisticalOutlierRemoval",
      ClassDesc("pcl/StatisticalOutlierRemoval",
                "StatisticalOutlierRemoval",
                "nodelet::Nodelet",
                "pcl_ros",
                "StatisticalOutlierRemoval uses point neighborhood statistics to filter outlier data.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/RadiusOutlierRemoval",
      ClassDesc("pcl/RadiusOutlierRemoval",
                "RadiusOutlierRemoval",
                "nodelet::Nodelet",
                "pcl_ros",
                "RadiusOutlierRemoval uses point neighborhood statistics to filter outlier data.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/CropBox",
      ClassDesc("pcl/CropBox",
                "CropBox",
                "nodelet::Nodelet",
                "pcl_ros",
                "CropBox is a filter that allows the user to filter all the data inside of a given box.",
                "lib/libpcl_ros_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/NodeletMUX",
      ClassDesc("pcl/NodeletMUX",
                "NodeletMUX",
                "nodelet::Nodelet",
                "pcl_ros",
                "NodeletMUX represent a mux nodelet for PointCloud topics: it takes N (up to 8) input topics, and publishes all of them on one output topic.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/NodeletDEMUX",
      ClassDesc("pcl/NodeletDEMUX",
                "NodeletDEMUX",
                "nodelet::Nodelet",
                "pcl_ros",
                "NodeletDEMUX represent a demux nodelet for PointCloud topics: it publishes 1 input topic to N output topics.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PCDReader",
      ClassDesc("pcl/PCDReader",
                "PCDReader",
                "nodelet::Nodelet",
                "pcl_ros",
                "PCDReader reads in a PCD (Point Cloud Data) v.5 file from disk and converts it to a PointCloud message.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/BAGReader",
      ClassDesc("pcl/BAGReader",
                "BAGReader",
                "nodelet::Nodelet",
                "pcl_ros",
                "BAGReader reads in sensor_msgs/PointCloud2 messages from BAG files.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PCDWriter",
      ClassDesc("pcl/PCDWriter",
                "PCDWriter",
                "nodelet::Nodelet",
                "pcl_ros",
                "PCDWriter writes a PointCloud message to disk in a PCD (Point Cloud Data) v.5 file format.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PointCloudConcatenateFieldsSynchronizer",
      ClassDesc("pcl/PointCloudConcatenateFieldsSynchronizer",
                "PointCloudConcatenateFieldsSynchronizer",
                "nodelet::Nodelet",
                "pcl_ros",
                "PointCloudConcatenateFieldsSynchronizer is a special form of data synchronizer: it listens for a set of input PointCloud messages on the same topic, checks their timestamps, and concatenates their fields together into a single PointCloud output message.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/PointCloudConcatenateDataSynchronizer",
      ClassDesc("pcl/PointCloudConcatenateDataSynchronizer",
                "PointCloudConcatenateDataSynchronizer",
                "nodelet::Nodelet",
                "pcl_ros",
                "PointCloudConcatenateDataSynchronizer is a special form of data synchronizer: it listens for a set of input PointCloud messages on different topics, and concatenates them together into a single PointCloud output message.",
                "lib/libpcl_ros_io",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/ExtractPolygonalPrismData",
      ClassDesc("pcl/ExtractPolygonalPrismData",
                "ExtractPolygonalPrismData",
                "nodelet::Nodelet",
                "pcl_ros",
                "ExtractPolygonalPrismData uses a set of point indices that represent a planar model, and together with a given height, generates a 3D polygonal prism. The polygonal prism is then used to segment all points lying inside it.",
                "lib/libpcl_ros_segmentation",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/EuclideanClusterExtraction",
      ClassDesc("pcl/EuclideanClusterExtraction",
                "EuclideanClusterExtraction",
                "nodelet::Nodelet",
                "pcl_ros",
                "EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense.",
                "lib/libpcl_ros_segmentation",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/SACSegmentationFromNormals",
      ClassDesc("pcl/SACSegmentationFromNormals",
                "SACSegmentationFromNormals",
                "nodelet::Nodelet",
                "pcl_ros",
                "SACSegmentation represents the Nodelet segmentation class for Sample Consensus methods and models, in the sense that it just creates a Nodelet wrapper for generic-purpose SAC-based segmentation.",
                "lib/libpcl_ros_segmentation",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/SACSegmentation",
      ClassDesc("pcl/SACSegmentation",
                "SACSegmentation",
                "nodelet::Nodelet",
                "pcl_ros",
                "SACSegmentation represents the Nodelet segmentation class for Sample Consensus methods and models, in the sense that it just creates a Nodelet wrapper for generic-purpose SAC-based segmentation.",
                "lib/libpcl_ros_segmentation",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/SegmentDifferences",
      ClassDesc("pcl/SegmentDifferences",
                "SegmentDifferences",
                "nodelet::Nodelet",
                "pcl_ros",
                "SegmentDifferences obtains the difference between two spatially aligned point clouds and returns the difference between them for a maximum given distance threshold.",
                "lib/libpcl_ros_segmentation",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/MovingLeastSquares",
      ClassDesc("pcl/MovingLeastSquares",
                "MovingLeastSquares",
                "nodelet::Nodelet",
                "pcl_ros",
                "MovingLeastSquares is an implementation of the MLS algorithm for data reconstruction through bivariate polynomial fitting.",
                "lib/libpcl_ros_surface",
                "")));
  pluginClasses.insert(
    plugin_pair("pcl/ConvexHull2D",
      ClassDesc("pcl/ConvexHull2D",
                "ConvexHull2D",
                "nodelet::Nodelet",
                "pcl_ros",
                "ConvexHull2D represents a 2D ConvexHull implementation.",
                "lib/libpcl_ros_surface",
                "")));
  pluginClasses.insert(
    plugin_pair("diagnostic_aggregator/GenericAnalyzer",
      ClassDesc("diagnostic_aggregator/GenericAnalyzer",
                "diagnostic_aggregator::GenericAnalyzer",
                "diagnostic_aggregator::Analyzer",
                "diagnostic_aggregator",
                "GenericAnalyzer is default diagnostic analyzer.",
                "lib/libdiagnostic_aggregator",
                "")));
  pluginClasses.insert(
    plugin_pair("diagnostic_aggregator/DiscardAnalyzer",
      ClassDesc("diagnostic_aggregator/DiscardAnalyzer",
                "diagnostic_aggregator::DiscardAnalyzer",
                "diagnostic_aggregator::Analyzer",
                "diagnostic_aggregator",
                "DiscardAnalyzer will discard (not report) any values that it matches.",
                "lib/libdiagnostic_aggregator",
                "")));
  pluginClasses.insert(
    plugin_pair("diagnostic_aggregator/IgnoreAnalyzer",
      ClassDesc("diagnostic_aggregator/IgnoreAnalyzer",
                "diagnostic_aggregator::IgnoreAnalyzer",
                "diagnostic_aggregator::Analyzer",
                "diagnostic_aggregator",
                "IgnoreAnalyzer will ignore all parameters and discard all.",
                "lib/libdiagnostic_aggregator",
                "")));
  pluginClasses.insert(
    plugin_pair("diagnostic_aggregator/AnalyzerGroup",
      ClassDesc("diagnostic_aggregator/AnalyzerGroup",
                "diagnostic_aggregator::AnalyzerGroup",
                "diagnostic_aggregator::Analyzer",
                "diagnostic_aggregator",
                "AnalyzerGroup is a way of grouping diagnostic analyzers. It is used internally by the aggregator, and can also be used as a plugin.",
                "lib/libdiagnostic_aggregator",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserArrayFilter",
      ClassDesc("laser_filters/LaserArrayFilter",
                "laser_filters::LaserArrayFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter which runs two internal MultiChannelFilterChain filters on the range and intensity measurements.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanIntensityFilter",
      ClassDesc("laser_filters/LaserScanIntensityFilter",
                "laser_filters::LaserScanIntensityFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter which filters sensor_msgs::LaserScan messages based on intensity",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanRangeFilter",
      ClassDesc("laser_filters/LaserScanRangeFilter",
                "laser_filters::LaserScanRangeFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter which filters sensor_msgs::LaserScan messages based on range",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/ScanShadowsFilter",
      ClassDesc("laser_filters/ScanShadowsFilter",
                "laser_filters::ScanShadowsFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter which filters points from a laser scan that look like the veiling effect.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/InterpolationFilter",
      ClassDesc("laser_filters/InterpolationFilter",
                "laser_filters::InterpolationFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter that will generate range readings for error readings in a scan by interpolating between valid readings on either side of the error",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanAngularBoundsFilter",
      ClassDesc("laser_filters/LaserScanAngularBoundsFilter",
                "laser_filters::LaserScanAngularBoundsFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter that removes points in a laser scan outside of certain angular bounds.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanAngularBoundsFilterInPlace",
      ClassDesc("laser_filters/LaserScanAngularBoundsFilterInPlace",
                "laser_filters::LaserScanAngularBoundsFilterInPlace",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter that removes points in a laser scan inside of certain angular bounds.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanBoxFilter",
      ClassDesc("laser_filters/LaserScanBoxFilter",
                "laser_filters::LaserScanBoxFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter that removes points in a laser scan inside of a cartesian box.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanMaskFilter",
      ClassDesc("laser_filters/LaserScanMaskFilter",
                "laser_filters::LaserScanMaskFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "This is a filter that removes points on directions defined in a mask from a laser scan.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserMedianFilter",
      ClassDesc("laser_filters/LaserMedianFilter",
                "laser_filters::LaserMedianFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "DEPRECATED: This is a median filter which filters sensor_msgs::LaserScan messages.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/LaserScanFootprintFilter",
      ClassDesc("laser_filters/LaserScanFootprintFilter",
                "laser_filters::LaserScanFootprintFilter",
                "filters::FilterBase<sensor_msgs::LaserScan>",
                "laser_filters",
                "DEPRECATED: This is a filter which filters points out of a laser scan which are inside the inscribed radius.",
                "lib/liblaser_scan_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("laser_filters/PointCloudFootprintFilter",
      ClassDesc("laser_filters/PointCloudFootprintFilter",
                "laser_filters::PointCloudFootprintFilter",
                "filters::FilterBase<sensor_msgs::PointCloud>",
                "laser_filters",
                "DEPRECATED: Remove points from the pointcloud inside the robot base.",
                "lib/libpointcloud_filters",
                "")));
  pluginClasses.insert(
    plugin_pair("base_local_planner/TrajectoryPlannerROS",
      ClassDesc("base_local_planner/TrajectoryPlannerROS",
                "base_local_planner::TrajectoryPlannerROS",
                "nav_core::BaseLocalPlanner",
                "base_local_planner",
                "A implementation of a local planner using either a DWA or Trajectory Rollout approach based on configuration parameters.",
                "lib/libtrajectory_planner_ros",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/theora_pub",
      ClassDesc("image_transport/theora_pub",
                "theora_image_transport::TheoraPublisher",
                "image_transport::PublisherPlugin",
                "theora_image_transport",
                "This plugin publishes a video packet stream encoded using Theora.",
                "lib/libtheora_image_transport",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/theora_sub",
      ClassDesc("image_transport/theora_sub",
                "theora_image_transport::TheoraSubscriber",
                "image_transport::SubscriberPlugin",
                "theora_image_transport",
                "This plugin decodes a video packet stream encoded using Theora.",
                "lib/libtheora_image_transport",
                "")));
  pluginClasses.insert(
    plugin_pair("image_proc/debayer",
      ClassDesc("image_proc/debayer",
                "image_proc::DebayerNodelet",
                "nodelet::Nodelet",
                "image_proc",
                "Nodelet to debayer (if needed) a raw camera image stream.",
                "lib/libimage_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_proc/rectify",
      ClassDesc("image_proc/rectify",
                "image_proc::RectifyNodelet",
                "nodelet::Nodelet",
                "image_proc",
                "Nodelet to rectify an unrectified camera image stream.",
                "lib/libimage_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_proc/resize",
      ClassDesc("image_proc/resize",
                "image_proc::ResizeNodelet",
                "nodelet::Nodelet",
                "image_proc",
                "Nodelet to resize image and camera_info.",
                "lib/libimage_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_proc/crop_decimate",
      ClassDesc("image_proc/crop_decimate",
                "image_proc::CropDecimateNodelet",
                "nodelet::Nodelet",
                "image_proc",
                "Nodelet to apply decimation (software binning) and ROI to a raw camera image post-capture.",
                "lib/libimage_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_proc/crop_nonZero",
      ClassDesc("image_proc/crop_nonZero",
                "image_proc::CropNonZeroNodelet",
                "nodelet::Nodelet",
                "image_proc",
                "",
                "lib/libimage_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_proc/crop_non_zero",
      ClassDesc("image_proc/crop_non_zero",
                "image_proc::CropNonZeroNodelet",
                "nodelet::Nodelet",
                "image_proc",
                "Nodelet to crop the largest valid (Non Zero) area of the image.",
                "lib/libimage_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_view/image",
      ClassDesc("image_view/image",
                "image_view::ImageNodelet",
                "nodelet::Nodelet",
                "image_view",
                "Nodelet to view a sensor_msgs/Image topic",
                "lib/libimage_view",
                "")));
  pluginClasses.insert(
    plugin_pair("image_view/disparity",
      ClassDesc("image_view/disparity",
                "image_view::DisparityNodelet",
                "nodelet::Nodelet",
                "image_view",
                "Nodelet to view a stereo_msgs/DisparityImage topic",
                "lib/libimage_view",
                "")));
  pluginClasses.insert(
    plugin_pair("image_rotate/image_rotate",
      ClassDesc("image_rotate/image_rotate",
                "image_rotate::ImageRotateNodelet",
                "nodelet::Nodelet",
                "image_rotate",
                "Nodelet to rotate sensor_msgs/Image",
                "lib/libimage_rotate",
                "")));
  pluginClasses.insert(
    plugin_pair("image_publisher/image_publisher",
      ClassDesc("image_publisher/image_publisher",
                "image_publisher::ImagePublisherNodelet",
                "nodelet::Nodelet",
                "image_publisher",
                "Nodelet to publish sensor_msgs/Image",
                "lib/libimage_publisher",
                "")));
  pluginClasses.insert(
    plugin_pair("stereo_image_proc/disparity",
      ClassDesc("stereo_image_proc/disparity",
                "stereo_image_proc::DisparityNodelet",
                "nodelet::Nodelet",
                "stereo_image_proc",
                "Nodelet to perform stereo processing on a pair of rectified image streams, producing disparity images",
                "lib/libstereo_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("stereo_image_proc/point_cloud2",
      ClassDesc("stereo_image_proc/point_cloud2",
                "stereo_image_proc::PointCloud2Nodelet",
                "nodelet::Nodelet",
                "stereo_image_proc",
                "Nodelet to produce XYZRGB PointCloud2 messages",
                "lib/libstereo_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/convert_metric",
      ClassDesc("depth_image_proc/convert_metric",
                "depth_image_proc::ConvertMetricNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to convert raw uint16 depth image in mm to float depth image in m.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/disparity",
      ClassDesc("depth_image_proc/disparity",
                "depth_image_proc::DisparityNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to convert depth image to disparity image.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/point_cloud_xyz",
      ClassDesc("depth_image_proc/point_cloud_xyz",
                "depth_image_proc::PointCloudXyzNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to convert depth image to XYZ point cloud.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/point_cloud_xyzrgb",
      ClassDesc("depth_image_proc/point_cloud_xyzrgb",
                "depth_image_proc::PointCloudXyzrgbNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to combine registered depth image and RGB image into XYZRGB point cloud.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/point_cloud_xyzi",
      ClassDesc("depth_image_proc/point_cloud_xyzi",
                "depth_image_proc::PointCloudXyziNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to combine registered depth image and intensity image into XYZI point cloud.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/point_cloud_xyz_radial",
      ClassDesc("depth_image_proc/point_cloud_xyz_radial",
                "depth_image_proc::PointCloudXyzRadialNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to convert an Radial depth map to a point.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/point_cloud_xyzi_radial",
      ClassDesc("depth_image_proc/point_cloud_xyzi_radial",
                "depth_image_proc::PointCloudXyziRadialNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to convert an Radial depth and intensity map to a point.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/register",
      ClassDesc("depth_image_proc/register",
                "depth_image_proc::RegisterNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to create a depth image registered to another camera frame.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("depth_image_proc/crop_foremost",
      ClassDesc("depth_image_proc/crop_foremost",
                "depth_image_proc::CropForemostNodelet",
                "nodelet::Nodelet",
                "depth_image_proc",
                "Nodelet to crop a depth image from foremost depth to a specific range.",
                "lib/libdepth_image_proc",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/compressedDepth_pub",
      ClassDesc("image_transport/compressedDepth_pub",
                "compressed_depth_image_transport::CompressedDepthPublisher",
                "image_transport::PublisherPlugin",
                "compressed_depth_image_transport",
                "This plugin publishes a compressed depth images using PNG compression.",
                "lib/libcompressed_depth_image_transport",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/compressedDepth_sub",
      ClassDesc("image_transport/compressedDepth_sub",
                "compressed_depth_image_transport::CompressedDepthSubscriber",
                "image_transport::SubscriberPlugin",
                "compressed_depth_image_transport",
                "This plugin decodes a compressed depth images.",
                "lib/libcompressed_depth_image_transport",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/compressed_pub",
      ClassDesc("image_transport/compressed_pub",
                "compressed_image_transport::CompressedPublisher",
                "image_transport::PublisherPlugin",
                "compressed_image_transport",
                "This plugin publishes a CompressedImage using either JPEG or PNG compression.",
                "lib/libcompressed_image_transport",
                "")));
  pluginClasses.insert(
    plugin_pair("image_transport/compressed_sub",
      ClassDesc("image_transport/compressed_sub",
                "compressed_image_transport::CompressedSubscriber",
                "image_transport::SubscriberPlugin",
                "compressed_image_transport",
                "This plugin decompresses a CompressedImage topic.",
                "lib/libcompressed_image_transport",
                "")));
  return pluginClasses;
}