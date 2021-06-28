/*
Warning!

In order to run this app an updated version of the cf firmware is needed: >2021.01
for the HLC functions

Adding the cf_messages package in the mcu_ws folder is neccessary to use the custom types
*/




#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "static_mem.h"
#include "system.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/u_int8.h>
#include <std_srvs/srv/trigger.h>
#include <geometry_msgs/msg/point32.h>

#include <cf_messages/msg/weather_data.h>
#include <cf_messages/msg/air_quality.h>
#include <cf_messages/msg/waypoint.h>
#include <cf_messages/msg/trajectory.h>
#include <cf_messages/msg/ping.h>

#include <cf_messages/srv/take_off.h>
#include <cf_messages/srv/land.h>
#include <cf_messages/srv/go_to.h>
#include <cf_messages/srv/stop.h>
#include <cf_messages/srv/simple_traj.h>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "crtp.h"
#include "radiolink.h"
#include "num.h"
#include "debug.h"
#include "configblock.h"
#include "param.h"
#include "crtp_commander_high_level.h"

#include "microrosapp.h"

#define UROS_DEBUG
#define STRING_BUFFER_LEN 50
#define PONG_MAX_LEN 1024

const char* node_name = "drone1";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){DEBUG_PRINT("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_publisher_t control_publisher;

rcl_subscription_t ping_subscriber;
rcl_subscription_t control_subscriber;
rcl_subscription_t pong_subscriber;

cf_messages__msg__Ping incoming_ping;
cf_messages__msg__Ping outcoming_ping;
std_msgs__msg__UInt8 control_ping;


//Basic instruction Services
cf_messages__srv__TakeOff_Request to_req;
cf_messages__srv__TakeOff_Response to_res;
cf_messages__srv__Land_Request land_req;
cf_messages__srv__Land_Response land_res;

int device_id;
int seq_no;
int pong_count;

struct timespec ts;



void ping_subscription_callback(const void * msgin)
{
	const cf_messages__msg__Ping * msg = (const cf_messages__msg__Ping *)msgin;

	//msg->data.size = 31;
	//DEBUG_PRINT("Ping received with seq %d. Answering.\n", (uint8_t)msg->data.data[0]);
	RCCHECK(rcl_publish(&pong_publisher, (const void*)msg, NULL));
	int64_t time = rmw_uros_epoch_millis();
	DEBUG_PRINT("Received at: %lld ms \n", time);
}


void control_ping_subscription_callback(const void * msgin)
{
	DEBUG_PRINT("HELLO WORLD\n");
	const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;

	DEBUG_PRINT("Ping received with seq %d. Answering.\n", (int)msg->data);
	DEBUG_PRINT("REceived at %lld\n", (int64_t)(rmw_uros_epoch_nanos()));

	//msg->data.size = 31;
	RCCHECK(rcl_publish(&control_publisher, (const void*)msg, NULL));
}

static void take_off_service_callback(const void *req, void *res){
	cf_messages__srv__TakeOff_Request *req_in = (cf_messages__srv__TakeOff_Request*) req;
	cf_messages__srv__TakeOff_Response *res_out = (cf_messages__srv__TakeOff_Response*) res;

	DEBUG_PRINT("REQ takeoff-h:%f dur:%f\n", (double)req_in->height, (double)req_in->duration);

	int ret = crtpCommanderHighLevelTakeoff(req_in->height, req_in->duration);
	res_out->ret = ret; 
}

static void land_service_callback(const void *req, void *res){
	cf_messages__srv__Land_Request *req_in = (cf_messages__srv__Land_Request*) req;
	cf_messages__srv__Land_Response *res_out = (cf_messages__srv__Land_Response*) res;

	DEBUG_PRINT("REQ land-h:%f dur:%f\n", (double)req_in->height, (double)req_in->duration);

	int ret = crtpCommanderHighLevelLand(req_in->height, req_in->duration);
	res_out->ret = ret; 
}


static paramVarId_t paramIdCommanderEnHighLevel;
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

void appMain(void)
{
    DEBUG_PRINT("START of APP\n");
	
	absoluteUsedMemory = 0;
    usedMemory = 0;


    //####################### RADIO INIT #######################
    int radio_connected = logGetVarId("radio", "isConnected");
    while(!logGetUint(radio_connected)) vTaskDelay(100);
    DEBUG_PRINT("Radio connected\n");

	vTaskDelay(1000/portTICK_RATE_MS);
	paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
	enableHighlevelCommander();
	DEBUG_PRINT("Enable HLC\n");


    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = __crazyflie_allocate;
    freeRTOS_allocator.deallocate = __crazyflie_deallocate;
    freeRTOS_allocator.reallocate = __crazyflie_reallocate;
    freeRTOS_allocator.zero_allocate = __crazyflie_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        DEBUG_PRINT("Error on default allocators (line %d)\n",__LINE__);
        vTaskSuspend( NULL );
    }

    const uint8_t radio_channel = 80;
    rmw_uros_set_custom_transport ( 
        true, 
    	(void *) &radio_channel, 
        crazyflie_serial_open, 
        crazyflie_serial_close, 
        crazyflie_serial_write, 
        crazyflie_serial_read
    ); 

    
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

	// Create a default ping publisher
	RCCHECK(rclc_publisher_init_default (&pong_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(cf_messages, msg, Ping), "/microROS/pong"));

	// Create a control ping publisher
	RCCHECK(rclc_publisher_init_default (&control_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/microROS/control_pong"));

	// Create a default ping subscriber
	RCCHECK(rclc_subscription_init_default (&ping_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(cf_messages, msg, Ping), "/microROS/ping"));

	// Create a default ping subscriber
	RCCHECK(rclc_subscription_init_default (&control_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/microROS/control_ping"));


	DEBUG_PRINT("CREATING SERVICES\n");

	// create landing service 
	rcl_service_t landing_service;
	RCCHECK(rclc_service_init_default(&landing_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(cf_messages, srv, Land), "/droneX/land"));

	// create takeoff service 
	rcl_service_t take_off_service;
	RCCHECK(rclc_service_init_default(&take_off_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(cf_messages, srv, TakeOff), "/droneX/takeoff"));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
	//RCCHECK(rclc_executor_add_subscription(&executor, &control_subscriber, &control_ping,
	//	&control_ping_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
		&ping_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_service(&executor, &take_off_service, &to_req, &to_res, take_off_service_callback));
	RCCHECK(rclc_executor_add_service(&executor, &landing_service, &land_req, &land_res, land_service_callback));


	incoming_ping.data.data = (uint8_t*) malloc(PONG_MAX_LEN*sizeof(uint8_t));
	incoming_ping.data.size = 0;
	incoming_ping.data.capacity = PONG_MAX_LEN;

	RCSOFTCHECK(rmw_uros_sync_session(1000));

	DEBUG_PRINT("uROS setup finished\n");


	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
	//	vTaskDelay(10/portTICK_RATE_MS);
	//rclc_executor_spin(&executor);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
	RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
	RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
