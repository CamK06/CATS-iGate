#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <time.h>
#include <flog.h>
#include <INIReader.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>

#include <grpcpp/grpcpp.h>	
#include <grpcpp/client_context.h>
#include "proto/cats.pb.h"
#include "proto/cats.grpc.pb.h"

#include "uuid_v4.h"

extern "C" {
#define BUILD_RADIO_IFACE
#include "cats/radio_iface.h"
#include "cats/packet.h"
#include "cats/error.h"
}

struct Config {
	std::string felinet_address;
	std::string callsign;
	std::string comment;
	std::string serial_port;
	int ssid;
	int icon;
	int beacon_interval;
	int altitude;
	int antenna_height;
	int heard_seconds;
	double latitude;
	double longitude;
	float antenna_gain;
	bool ignore_aprs;
} config;

static int radio_fd;
static std::shared_ptr<grpc::Channel> felinet_channel;
static std::unique_ptr<felinet::Handler::Stub> felinet_stub;
static UUIDv4::UUID felinet_uuid;
static const uint8_t CBOR_BEGIN[3] = { 0xd9, 0xd9, 0xf7 };
static std::thread beacon_thread;
static std::thread felinet_thread;
static bool beacon_running;
static bool felinet_running;
static time_t beacon_time;
static time_t startup_time;
static time_t last_rx;

void hexdump(uint8_t* data, size_t len)
{
	for(int i = 0; i < len; i++) {
		printf("%02X ", data[i]);
	}
	printf("\n");
}

void config_load()
{
	INIReader reader("config.ini");
	if(reader.ParseError() < 0) {
		flog::error("Failed to load config");
		flog::error("Quitting...");
		std::exit(-1);
	}
	config.felinet_address = reader.Get("connection", "felinet_address", "https://felinet.cats.radio");
	config.callsign = reader.Get("beacon", "callsign", "n0call");
	config.comment = reader.Get("beacon", "comment", "Hello CATS world!");
	config.serial_port = reader.Get("connection", "serial_port", "/dev/null");
	config.ssid = reader.GetInteger("beacon", "ssid", 0);
	config.icon = reader.GetInteger("beacon", "icon", 0);
	config.beacon_interval = reader.GetInteger("beacon", "interval", 0);
	config.altitude = reader.GetInteger("beacon.position", "altitude", 0);
	config.antenna_height = reader.GetInteger("beacon", "antenna_height", 0);
	config.heard_seconds = reader.GetInteger("connection", "felinet_heard_seconds", 900);
	config.latitude = reader.GetReal("beacon.position", "latitude", 0);
	config.longitude = reader.GetReal("beacon.position", "longitude", 0);
	config.antenna_gain = reader.GetReal("beacon", "antenna_gain", 0);
	config.ignore_aprs = reader.GetBoolean("connection", "ignore_aprs", true);
}

void print_packet(cats_packet_t* pkt)
{
    cats_whisker_data_t* data;
    if(cats_packet_get_identification(pkt, (cats_ident_whisker_t**)&data) == CATS_SUCCESS) {
        printf("IDENT: \t%s-%d [ICON: %d]\n", 
                data->identification.callsign, 
                data->identification.ssid, 
                data->identification.icon
        );
    }
    if(cats_packet_get_route(pkt, (cats_route_whisker_t**)&data) == CATS_SUCCESS) {
        printf("ROUTE: \t(MAX %d) ", data->route.max_digipeats);
        cats_route_hop_t* hop = &(data->route.hops);
        while(hop != NULL) {
            if(hop->hop_type == CATS_ROUTE_INET) {
                printf("[NET]");
            }
            else if(hop->hop_type == CATS_ROUTE_FUTURE) {
                printf("%s-%d*", hop->callsign, hop->ssid);
            }
            else if(hop->hop_type == CATS_ROUTE_PAST) {
                printf("%s-%d [%.1f dBm]", hop->callsign, hop->ssid, hop->rssi);
            }
            if(hop->next != NULL) {
                printf(" -> ");
            }
            hop = hop->next;
        }
        printf("\n");
    }
    char comment[CATS_MAX_PKT_LEN];
    if(cats_packet_get_comment(pkt, comment) == CATS_SUCCESS) {
        printf("CMNT: \t'%s'\n", comment);
    }
    if(cats_packet_get_gps(pkt, (cats_gps_whisker_t**)&data) == CATS_SUCCESS) {
        printf("GPS: \t(%.4f, %.4f) +/- %d m, v = %.2f m/s [N %.2f] deg\nALT: \t%.2f m\n",
                data->gps.latitude,
                data->gps.longitude,
                data->gps.max_error,
                data->gps.speed,
                data->gps.heading,
                data->gps.altitude
        );
    }
    if(cats_packet_get_nodeinfo(pkt, (cats_nodeinfo_whisker_t**)&data) == CATS_SUCCESS) {
        if(data->node_info.hardware_id.enabled && data->node_info.software_id.enabled) {
            printf("HW: \t0x%04x SW: 0x%02x\n", data->node_info.hardware_id.val, data->node_info.software_id.val);
        }
        else if(data->node_info.hardware_id.enabled) {
            printf("HW: \t0x%04x\n", data->node_info.hardware_id.val);
        }
        else if(data->node_info.software_id.enabled) {
            printf("SW: \t0x%02x\n", data->node_info.software_id.val);
        }

        if(data->node_info.uptime.enabled) {
            printf("UTIME: \t%d s\n", data->node_info.uptime.val);
        }
        if(data->node_info.ant_height.enabled) {
            printf("VERT: \t%d m\n", data->node_info.ant_height.val);
        }
        if(data->node_info.ant_gain.enabled) {
            printf("GAIN: \t%.2f dBi\n", data->node_info.ant_gain.val);
        }
        if(data->node_info.tx_power.enabled) {
            printf("TXP: \t%d dBm\n", data->node_info.tx_power.val);
        }
        if(data->node_info.voltage.enabled) {
            printf("VOLTS: \t%d V\n", data->node_info.voltage.val);
        }
        if(data->node_info.temperature.enabled) {
            printf("TEMP: \t%d C\n", data->node_info.temperature.val);
        }
    }
	cats_whisker_t** arbitrary;
	if((cats_packet_get_arbitrary(pkt, &arbitrary) != CATS_FAIL)
	&& arbitrary[0]->data.raw[0] == 0xc0) {
		printf("SRC: \tAPRS\n");
	}
	else {
		printf("SRC: \tCATS\n");
	}
}

bool felinet_send(cats_packet_t* pkt) 
{
	uint8_t buffer[CATS_MAX_PKT_LEN];
	int len = cats_packet_semi_encode(pkt, buffer);
	if(len == CATS_FAIL) {
		flog::error("Failed to encode packet");
		return false;
	}

	felinet::PacketIn out;
	out.set_raw(buffer, len);
	out.set_uuid(felinet_uuid.bytes());

	grpc::ClientContext ctx;
	felinet::PushPacketResponse response;
	response.Clear();
	auto push = felinet_stub->PushPackets(&ctx, &response);
	if(!push->Write(out)) {
		flog::error("Failed to push packet to felinet");
		grpc::Status status = push->Finish();
		flog::error(status.error_message().c_str());
	}
	push->WritesDone();
	grpc::Status status = push->Finish();
	if(status.ok()) {
		//flog::debug("Packet successfully pushed to felinet");
	}
	else {
		flog::error("Failed to push packet to felinet");
		flog::error(status.error_message().c_str());
	}
	return true;
}

bool radio_send(cats_packet_t* pkt)
{
	uint8_t buffer[CATS_MAX_PKT_LEN];
	int len = cats_packet_semi_encode(pkt, buffer);
	if(len == CATS_FAIL) {
		flog::error("Failed to encode packet");
		return false;
	}
	len = cats_radio_iface_encode(buffer, len, 0);
	if(write(radio_fd, buffer, len) < 0) {
		flog::error("Failed to write to serial port");
		return false;
	}
	return true;
}

void start_felinet()
{
	std::string address = config.felinet_address;
	if(address.find("https://") != std::string::npos) {
		address = config.felinet_address.substr(8);
		felinet_channel = grpc::CreateChannel(address, grpc::SslCredentials(grpc::SslCredentialsOptions()));
	}
	else { 
		if(address.find("http://") != std::string::npos) {
			address = config.felinet_address.substr(7);
		}
		felinet_channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
	}
	felinet_stub = felinet::Handler::NewStub(felinet_channel);

	UUIDv4::UUIDGenerator<std::mt19937_64> uuid_generator;
	felinet_uuid = uuid_generator.getUUID();
	flog::info("Connected to felinet");
}

void open_serial()
{
	radio_fd = open(config.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if(radio_fd < 0) {
		flog::error("Failed to open serial port {}", config.serial_port);
		std::exit(-1);
	}

	struct termios tty;
	std::memset(&tty, 0, sizeof(tty));
	if(tcgetattr(radio_fd, &tty) != 0) {
		flog::error("Failed to fetch serial port attributes for port {}", config.serial_port);
		std::exit(-1);
	}
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tty.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    tty.c_cflag &= ~(CSIZE | PARENB);
    tty.c_cflag |= CS8;

	if(tcsetattr(radio_fd, TCSAFLUSH, &tty) != 0) {
		flog::error("Failed to set serial port attributes");
		std::exit(-1);
	}

	flog::info("Opened serial port {}", config.serial_port);
}

void beacon_worker()
{
	while(beacon_running) {
		double elapsed = difftime(time(NULL), beacon_time);
		if(elapsed >= config.beacon_interval) {
			cats_packet_t* pkt;
			cats_packet_prepare(&pkt);
			cats_packet_add_identification(pkt, config.callsign.c_str(), config.ssid, config.icon);
			cats_packet_add_comment(pkt, config.comment.c_str());
			cats_packet_add_gps(pkt, config.latitude, config.longitude, config.altitude, 0, 0, 0);
			cats_nodeinfo_whisker_t info;
			info.ant_gain.enabled = true;
        	info.ant_height.enabled = true;
        	info.battery_level.enabled = false;
        	info.software_id.enabled = true;
        	info.temperature.enabled = false;
        	info.voltage.enabled = false;
        	info.tx_power.enabled = false;
        	info.uptime.enabled = true;
        	info.hardware_id.enabled = false;
			info.ambient_humidity.enabled = false;
			info.ambient_pressure.enabled = false;
			info.ambient_temp.enabled = false;
			info.altitude.enabled = false;
        	info.ant_gain.val = config.antenna_gain;
			info.ant_height.val = config.antenna_height;
        	info.uptime.val = difftime(time(NULL), startup_time);
        	info.software_id.val = 0;
        	cats_packet_add_nodeinfo(pkt, info);
			cats_route_whisker_t route = cats_route_new(3);
			cats_packet_add_route(pkt, route);

			felinet_send(pkt);
			radio_send(pkt);
			cats_packet_destroy(&pkt);

			flog::debug("Beacon sent");
			printf("\n");
			time(&beacon_time);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}

void felinet_worker()
{
	felinet::PacketOut out;
	felinet::PacketRequest request;
	request.set_uuid(felinet_uuid.bytes());
	request.add_filters()->set_heard_seconds(config.heard_seconds);
	//felinet::DistanceFilter dist;
	//dist.set_latitude(config.latitude);
	//dist.set_longitude(config.longitude);
	//dist.set_distance(100000);
	//request.add_filters()->set_allocated_dist(&dist);

	grpc::ClientContext ctx;
	auto stream = felinet_stub->GetPackets(&ctx, request);
	flog::info("Listening for felinet packets");
	uint8_t data[CATS_MAX_PKT_LEN];
	while(stream->Read(&out)) {
		cats_packet_t* pkt;
		cats_packet_prepare(&pkt);
		if(cats_packet_semi_decode(pkt, (uint8_t*)out.raw().c_str(), out.raw().size()) == CATS_FAIL) {
			flog::error("Failed to decode packet");
			cats_packet_destroy(&pkt);
			continue;
		}
		
		cats_ident_whisker_t* ident;
		int r = cats_packet_get_identification(pkt, &ident);
		if(r != CATS_FAIL && strcmp((char*)ident->callsign, config.callsign.c_str()) == 0 && ident->ssid == config.ssid) {
			cats_packet_destroy(&pkt);
			continue;
		}

		cats_whisker_t** arbitrary;
		if((cats_packet_get_arbitrary(pkt, &arbitrary) != CATS_FAIL)
		&& arbitrary[0]->data.raw[0] == 0xc0 && config.ignore_aprs) {
			cats_packet_destroy(&pkt);
			continue;
		}

		flog::info("Felinet RX:");
		print_packet(pkt);
		printf("\n\n");
		
		cats_route_whisker_t* route;
		if(cats_packet_get_route(pkt, &route) == CATS_FAIL) {
			cats_packet_destroy(&pkt);
			continue;
		}
		cats_route_add_past_hop(route, config.callsign.c_str(), config.ssid, 0);

		if(!radio_send(pkt)) {
			flog::error("Failed to send packet to radio");
		}
		flog::debug("Felinet packet sent to radio");
		cats_packet_destroy(&pkt);
		printf("\n");
	}

	felinet_running = false;
}

int main()
{
	flog::info("Starting iGate...");
	config_load();
	open_serial();
	start_felinet();

	time(&beacon_time);
	time(&startup_time);
	time(&last_rx);
	beacon_thread = std::thread(&beacon_worker);
	beacon_running = true;
	felinet_thread = std::thread(&felinet_worker);
	felinet_running = true;

	uint8_t serial_buf[CATS_MAX_PKT_LEN];
	uint8_t pkt_buf[CATS_MAX_PKT_LEN];
	int bytes_read = 0;
	while(felinet_running) {
		int n = read(radio_fd, serial_buf, CATS_MAX_PKT_LEN);
		if(n < 0) {
			continue;
		}

		if(difftime(time(NULL), last_rx) >= 3) {
			memset(pkt_buf, 0x00, CATS_MAX_PKT_LEN);
			bytes_read = 0;
			//flog::debug("Serial timeout");
		}
		memcpy(pkt_buf + bytes_read, serial_buf, n);
		memset(serial_buf, 0x00, CATS_MAX_PKT_LEN);
		time(&last_rx);

		bytes_read += n;
		if(bytes_read < 3) {
			continue;
		}

		if(memcmp(pkt_buf, CBOR_BEGIN, 3) != 0) {
			//flog::error("Invalid packet");
			bytes_read = 0;
			memset(pkt_buf, 0, CATS_MAX_PKT_LEN);
			continue;
		}


		float rssi = 0;
		int r = cats_radio_iface_decode(pkt_buf, bytes_read, &rssi);
		if(r == CATS_FAIL) {
			continue;
		}
		bytes_read = r;

		//flog::debug("Read {} bytes from radio", bytes_read);

		cats_packet_t* pkt;
		cats_packet_prepare(&pkt);
		if(cats_packet_semi_decode(pkt, pkt_buf, bytes_read) == CATS_FAIL) {
			flog::error("Failed to decode packet from radio");
			cats_packet_destroy(&pkt);
			memset(pkt_buf, 0, CATS_MAX_PKT_LEN);
			bytes_read = 0;
			continue;
		}
		memset(pkt_buf, 0x00, CATS_MAX_PKT_LEN);
		bytes_read = 0;

		flog::info("Radio RX [{} dbm]:", rssi);
		print_packet(pkt);

		cats_route_whisker_t* route;
		if(cats_packet_get_route(pkt, &route) == CATS_FAIL) {
			flog::debug("Failed to get route");
			cats_packet_destroy(&pkt);
			continue;
		}

		// Digipeat the packet
		int new_len = 0;
		cats_route_add_past_hop(route, config.callsign.c_str(), config.ssid, rssi);
		if(cats_packet_should_digipeat(pkt, config.callsign.c_str(), config.ssid)) {
			if(!radio_send(pkt)) {
				flog::error("Failed to digipeat packet");
				cats_packet_destroy(&pkt);
				continue;
			}
			flog::debug("Packet digipeated");
		}
	
		// iGate the packet
		cats_route_add_inet_hop(route);
		if(!felinet_send(pkt)) {
			flog::error("Failed to igate packet");
		}

		cats_packet_destroy(&pkt);
		printf("\n");
	}
}
