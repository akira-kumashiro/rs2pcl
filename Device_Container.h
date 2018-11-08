#pragma once

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>                    // std::ceil

class Device_Container
{
	// Helper struct per pipeline
	struct view_port
	{
		std::map<int, rs2::frame> frames_per_stream;
		rs2::colorizer colorize_frame;
		texture tex;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
	};
private:
	const std::string no_camera_message = "No camera connected, please connect 1 or more";
	const std::string platform_camera_name = "Platform Camera";

public:
	void enable_device(rs2::device dev);
	void remove_devices(const rs2::event_information& info);
	size_t device_count();
	int stream_count();

	void poll_frames()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over all device
		for (auto&& view : _devices)
		{
			// Ask each pipeline if there are new frames available
			rs2::frameset frameset;
			if (view.second.pipe.poll_for_frames(&frameset))
			{
				for (int i = 0; i < frameset.size(); i++)
				{
					rs2::frame new_frame = frameset[i];
					int stream_id = new_frame.get_profile().unique_id();
					//auto value = view.second.colorize_frame(new_frame);

					//view.second.frames_per_stream[stream_id] = view.second.colorize_frame(new_frame); //update view port with the new stream
				}
			}
		}
	}
	//	void render_textures(int cols, int rows, float view_width, float view_height)
	//	{
	//std::lock_guard<std::mutex> lock(_mutex);
	//		int stream_no = 0;
	//		for (auto&& view : _devices)
	//		{
	//			// For each device get its frames
	//			for (auto&& id_to_frame : view.second.frames_per_stream)
	//			{
	//				// If the frame is available
	//				if (id_to_frame.second)
	//				{
	//					view.second.tex.upload(id_to_frame.second);
	//				}
	//				rect frame_location{ view_width * (stream_no % cols), view_height * (stream_no / cols), view_width, view_height };
	//				if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>())
	//				{
	//					rect adjuested = frame_location.adjust_ratio({ static_cast<float>(vid_frame.get_width())
	//																 , static_cast<float>(vid_frame.get_height()) });
	//					view.second.tex.show(adjuested);
	//					stream_no++;
	//				}
	//			}
	//		}
	//	}
private:
	std::mutex _mutex;
	std::map<std::string, view_port> _devices;
};

