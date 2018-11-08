#include "Device_Container.h"

//Device_Container::Device_Container()
//{
//}
//
//
//Device_Container::~Device_Container()
//{
//}

void Device_Container::enable_device(rs2::device dev)
{
	std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	std::lock_guard<std::mutex> lock(_mutex);

	if (_devices.find(serial_number) != _devices.end())
	{
		return; //already in
	}

	// Ignoring platform cameras (webcams, etc..)
	if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
	{
		return;
	}
	// Create a pipeline from the given device
	rs2::pipeline p;
	rs2::config c;
	c.enable_device(serial_number);
	// Start the pipeline with the configuration
	rs2::pipeline_profile profile = p.start(c);
	// Hold it internally
	_devices.emplace(serial_number, view_port{ {},{},{}, p, profile });

}

void Device_Container::remove_devices(const rs2::event_information & info)
{
	std::lock_guard<std::mutex> lock(_mutex);
	// Go over the list of devices and check if it was disconnected
	auto itr = _devices.begin();
	while (itr != _devices.end())
	{
		if (info.was_removed(itr->second.profile.get_device()))
		{
			itr = _devices.erase(itr);
		}
		else
		{
			++itr;
		}
	}
}

size_t Device_Container::device_count()
{
	std::lock_guard<std::mutex> lock(_mutex);
	return _devices.size();
}

int Device_Container::stream_count()
{
	std::lock_guard<std::mutex> lock(_mutex);
	int count = 0;
	for (auto&& sn_to_dev : _devices)
	{
		for (auto&& stream : sn_to_dev.second.frames_per_stream)
		{
			if (stream.second)
			{
				count++;
			}
		}
	}
	return count;
}
