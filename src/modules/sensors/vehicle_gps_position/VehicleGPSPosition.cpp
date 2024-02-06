/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

// Added for TCP Socket 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <pthread.h>

#include <string>
#include <sstream>
#include <vector>
#include <iterator>



namespace sensors
{

bool spoof_altitude = false;

float spoofed_lat = 0.0;
float spoofed_lon = 0.0;
float spoofed_alt = 0.0;

// Define the function to setup and run the TCP server
void* run_tcp_server(void* arg) {

	PX4_INFO("STARTING TCP SERVER");

    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        PX4_ERR("socket failed");
        pthread_exit(NULL);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(14552);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        PX4_ERR("bind failed");
        pthread_exit(NULL);
    }

    if (listen(server_fd, 3) < 0) {
        PX4_ERR("listen");
        pthread_exit(NULL);
    }

    while(true){
		if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
			perror("accept");
			pthread_exit(NULL);
		}

		memset(buffer, 0, sizeof(buffer));
		int valread = read(new_socket, buffer, 1024);
		if(valread > 0) {
			std::string command(buffer);
			PX4_INFO("RECEIVED A COMMAND: %s", command.c_str());

			// Split the command by spaces
			std::istringstream iss(command);
			std::vector<std::string> tokens(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());

			// Check if it's a start command with latitude, longitude and altitude
			if(tokens.size() == 4 && tokens[0] == "start") {
				
					// Convert lat, lon, alt to float
					spoofed_lat = std::stof(tokens[1]);
					spoofed_lon = std::stof(tokens[2]);
					spoofed_alt = std::stof(tokens[3]);

					// Set lat, lon, alt somewhere (depending on your software architecture)
					// Here, I'll just print them out
					PX4_INFO("Lat: %f", static_cast<double>(spoofed_lat));
					PX4_INFO("Lon: %f", static_cast<double>(spoofed_lon));
					PX4_INFO("Alt: %f", static_cast<double>(spoofed_alt));
					spoof_altitude = true;
			
			} else if(tokens[0] == "stop") {
				PX4_INFO("SPOOFING STOPPED");
				spoof_altitude = false;
			}
		}

		close(new_socket);
	}
	return nullptr;
}

/*
// Implement the StartTcpServer method.
void VehicleGPSPosition::StartTcpServer()
{
    pthread_t spoof_thread;
    if(pthread_create(&spoof_thread, NULL, TcpListen, NULL) < 0) {
        PX4_ERR("Could not create spoofing thread");
    }
}

// Implement the TcpListen method.
void* VehicleGPSPosition::TcpListen(void* arg)
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        PX4_ERR("socket failed");
        pthread_exit(NULL);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(14551);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        PX4_ERR("bind failed");
        pthread_exit(NULL);
    }

    if (listen(server_fd, 3) < 0) {
        PX4_ERR("listen");
        pthread_exit(NULL);
    }

    while(true){
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            pthread_exit(NULL);
        }

        memset(buffer, 0, sizeof(buffer));
        int valread = read(new_socket, buffer, 1024);
        if(valread > 0) {
            std::string command(buffer);
            PX4_INFO("RECEIVED A COMMAND: %s", command.c_str());
            
            if(command == "start") {
                PX4_INFO("SPOOFING STARTED");
                spoof_altitude = true;
            } else if(command == "stop") {
                PX4_INFO("SPOOFING STOPPED");
                spoof_altitude = false;
            }
        }

        close(new_socket);
    }

    return NULL;
}*/

VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	pthread_t server_thread;
    pthread_create(&server_thread, NULL, run_tcp_server, NULL);
	_vehicle_gps_position_pub.advertise();
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);
	//StartTcpServer();  // Start the TCP server when the module starts.
	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps_data);
			_gps_blending.setGpsData(gps_data, i);

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		sensor_gps_s gps_output = {_gps_blending.getOutputGpsData()};

		// Print out the altitude
        //PX4_INFO("Altitude: %f", (double)gps_output.alt);

		
		

		if (_gps_blending.isNewOutputDataAvailable()) {
			Publish(gps_output, _gps_blending.getSelectedGps());
		}
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::Publish(const sensor_gps_s &gps, uint8_t selected)
{
	vehicle_gps_position_s gps_output{};

	gps_output.timestamp = gps.timestamp;
	gps_output.time_utc_usec = gps.time_utc_usec;
	gps_output.lat = gps.lat;
	gps_output.lon = gps.lon;
	gps_output.alt = gps.alt;
	gps_output.alt_ellipsoid = gps.alt_ellipsoid;
	gps_output.s_variance_m_s = gps.s_variance_m_s;
	gps_output.c_variance_rad = gps.c_variance_rad;
	gps_output.eph = gps.eph;
	gps_output.epv = gps.epv;
	gps_output.hdop = gps.hdop;
	gps_output.vdop = gps.vdop;
	gps_output.noise_per_ms = gps.noise_per_ms;
	gps_output.jamming_indicator = gps.jamming_indicator;
	gps_output.jamming_state = gps.jamming_state;
	gps_output.vel_m_s = gps.vel_m_s;
	gps_output.vel_n_m_s = gps.vel_n_m_s;
	gps_output.vel_e_m_s = gps.vel_e_m_s;
	gps_output.vel_d_m_s = gps.vel_d_m_s;
	gps_output.cog_rad = gps.cog_rad;
	gps_output.timestamp_time_relative = gps.timestamp_time_relative;
	gps_output.heading = gps.heading;
	gps_output.heading_offset = gps.heading_offset;
	gps_output.fix_type = gps.fix_type;
	gps_output.vel_ned_valid = gps.vel_ned_valid;
	gps_output.satellites_used = gps.satellites_used;

	if (spoof_altitude) {
		gps_output.alt = gps_output.alt + static_cast<int>(spoofed_alt*1000);
	}

	gps_output.selected = selected;

	_vehicle_gps_position_pub.publish(gps_output);
}

void VehicleGPSPosition::PrintStatus()
{
	//PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_select_index);
}

}; // namespace sensors
