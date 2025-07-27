
#define NOMINMAX
#include <iostream>
#include <string>

#include "benewake_common.h"
#include "parse_pcap.h"

#ifdef _WIN32
#include <conio.h>
#else
#include <termio.h>

#endif // _WIN32
benewake::ParsePcap *parse_cap_ = new benewake::ParsePcap();

int main()
{
	// NOTE: if data is from older short range Horn X2 lidar, uncommit following code
	// parse_cap_->setX2LongRangeMode(false);

	std::string name = "D:/BenewakeWorkspace/X-project/data/1.pcap";
	//Find frame offset in file than record it in frame_offset_list_
	parse_cap_->pcap_frame_find(name);

	std::cout << "frame size: " << parse_cap_->frame_offset_list_.size() << std::endl;
	
	if (parse_cap_->frame_offset_list_.size() == 0)
	{
		system("pause");
		return 0;
	}
	benewake::BwPointCloud::Ptr bw_pointcloud_ = nullptr;
	int frame_id = 1;
	parse_cap_->parseFrameByOffset(parse_cap_->frame_offset_list_.at(frame_id), bw_pointcloud_);
	if (bw_pointcloud_ == nullptr)
	{
		system("pause");
		return 0;
	}

	int points_amount = bw_pointcloud_->points.size();
	std::cout << "frame " << frame_id << ":\n  size " << points_amount << std::endl;
	if (points_amount > 0)
	{
		/*
		* Do some work ...
		*/

		printf("  begin at %d.%09d s\n", bw_pointcloud_->points[0].timestamp_s, bw_pointcloud_->points[0].timestamp_ns);
		printf("  end at %d.%09d s\n", bw_pointcloud_->points[points_amount - 1].timestamp_s, bw_pointcloud_->points[points_amount - 1].timestamp_ns);

		//for (int i = 0; i < points_amount; i++)
		//{
		//	printf("x: %f y: %f z: %f i: %d timestamp(s): %d.%d channel: %d line: %d\n", bw_pointcloud_->points[i].x,
		//		   bw_pointcloud_->points[i].y, bw_pointcloud_->points[i].z, bw_pointcloud_->points[i].intensity, bw_pointcloud_->points[i].timestamp_s,
		//		   bw_pointcloud_->points[i].timestamp_ns, bw_pointcloud_->points[i].channel, bw_pointcloud_->points[i].row);
		//}
	}
	system("pause");
	return 0;
}