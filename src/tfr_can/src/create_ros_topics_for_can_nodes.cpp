#include "master.h"
#include "bridge.h"
#include "logger.h"
#include "joint_state_publisher.h"
#include "joint_state_subscriber.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"

#include <thread>
#include <chrono>
#include <memory>

//#include <ros/package.h> // for looking up the location of the current package, in order to find our EDS files.
//#include <ros>

int main(int argc, char* argv[]) {

	// Set the name of your CAN bus. "slcan0" is a common bus name
	// for the first SocketCAN device on a Linux system.
	const std::string busname = "can1";

	// Set the baudrate of your CAN bus. Most drivers support the values
	// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
	const std::string baudrate = "1M";

	const size_t num_devices_required = 2;

	const double loop_rate = 4; // [Hz]

	
	kaco::Master master;
	if (!master.start(busname, baudrate)) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	while (master.num_devices()<num_devices_required) {
		ERROR("Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
		PRINT("Trying to discover more nodes via NMT Node Guarding...");
		master.core.nmt.discover_nodes();
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Create bridge
	ros::init(argc, argv, "canopen_bridge");
	kaco::Bridge bridge;

	for (size_t i=0; i<master.num_devices(); ++i) {

		kaco::Device& device = master.get_device(i);
		device.start();

		PRINT("Found device with node ID "<<device.get_node_id()<<": "<<device.get_entry("manufacturer_device_name"));

		if (device.get_node_id() == 8) {
			
			ros::NodeHandle nh{"~"};
			std::string eds_files_path;
			if (nh.getParam("eds_files_path", eds_files_path))
			{
				PRINT("Great it worked.");
				PRINT(eds_files_path);
			}
			else
			{
				ERROR("tfr_can could not find the private parameter 'eds_files_path'. Make sure this parameter is getting set in the launch file for tfr_can.");
			}
			
			//const std::string eds_files_path = "/src/tfr_can/eds_files/";
			device.load_dictionary_from_eds(eds_files_path + "roboteq_motor_controllers_v60.eds");
			

			auto iosub_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_1_1);

			auto iopub_1_1 = std::make_shared<kaco::EntryPublisher>(device, "qry_relcntr/channel_1");
    		bridge.add_publisher(iopub_1_1, loop_rate);

			auto iopub_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_batamps/channel_1");
    		bridge.add_publisher(iopub_1_2, loop_rate);

			auto iosub_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
    		bridge.add_subscriber(iosub_2_1);

			auto iopub_2_1 = std::make_shared<kaco::EntryPublisher>(device, "qry_relcntr/channel_2");
    		bridge.add_publisher(iopub_2_1, loop_rate);

			auto iopub_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_batamps/channel_2");
    		bridge.add_publisher(iopub_2_2, loop_rate);
		}

	}

	PRINT("About to call bridge.run()");
	while (ros::ok())
	{
		try
		{
			bridge.run();
		}
		catch (const std::exception& e)
		{
			PRINT(e.what() << std::endl);
		}
	}
	
	
	return EXIT_SUCCESS;
}
