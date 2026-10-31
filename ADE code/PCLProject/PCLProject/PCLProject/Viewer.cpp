#include "Viewer.h"

Viewer::Viewer()
{
}


Viewer::~Viewer()
{
}

void Viewer::showViewer()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(basic_cloud_ptr);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Viewer::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}