#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/point32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
//#include <cf_messages/msg/weather_data.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>

#include "config.h"
#include "log.h"
#include "crc.h"
#include "worker.h"
#include "num.h"
#include "debug.h"
#include <time.h>

#include "microrosapp.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_odometry;
rcl_publisher_t publisher_attitude;
rcl_publisher_t test_publisher;
rcl_publisher_t test_string_publisher;
//rcl_publisher_t weather_publisher;

static int pitchid, rollid, yawid;
static int Xid, Yid, Zid;

void appMain(){
    absoluteUsedMemory = 0;
    usedMemory = 0;

    // ####################### RADIO INIT #######################

    vTaskDelay(2000);
    int radio_connected = logGetVarId("radio", "isConnected");
    while(!logGetUint(radio_connected)) vTaskDelay(100);
    DEBUG_PRINT("Radio connected\n");

    // ####################### MICROROS INIT #######################
    DEBUG_PRINT("Free heap pre uROS: %d bytes\n", xPortGetFreeHeapSize());
    vTaskDelay(50);

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
    rmw_uros_set_custom_transport( 
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
	RCCHECK(rclc_node_init_default(&node, "crazyflie_node1", "", &support));

	// create publishers
	RCCHECK(rclc_publisher_init_best_effort(&publisher_odometry, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/drone1/odometry"));
	RCCHECK(rclc_publisher_init_best_effort(&publisher_attitude, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point32), "/drone1/attitude"));
    RCCHECK(rclc_publisher_init_best_effort(&test_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/drone1/int_test"));
    RCCHECK(rclc_publisher_init_best_effort(&test_string_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/drone1/string_test"));
    //RCCHECK(rclc_publisher_init_best_effort(&weather_publisher, &node,
    //    ROSIDL_GET_MSG_TYPE_SUPPORT(cf_messages, msg, WeatherData), "/drone1/weather_data"));

    // // Init messages
    geometry_msgs__msg__Point32 pose;
    geometry_msgs__msg__Point32 odom;
    std_msgs__msg__Int32 test_msg;
    std_msgs__msg__String test_str;
    //cf_messages__msg__WeatherData weather_msg;

    //Get pitch, roll and yaw value
    pitchid = logGetVarId("stateEstimate", "pitch");
    rollid = logGetVarId("stateEstimate", "roll");
    yawid = logGetVarId("stateEstimate", "yaw");

    //Get X,Y and Z value
    Xid = logGetVarId("stateEstimate", "x");
    Yid = logGetVarId("stateEstimate", "y");
    Zid = logGetVarId("stateEstimate", "z");

    DEBUG_PRINT("Free heap post uROS configuration: %d bytes\n", xPortGetFreeHeapSize());
    DEBUG_PRINT("uROS Used Memory %d bytes\n", usedMemory);
    DEBUG_PRINT("uROS Absolute Used Memory %d bytes\n", absoluteUsedMemory);

    test_str.data.data = (char * ) malloc(50 * sizeof(char));
	test_str.data.size = 0;
	test_str.data.capacity = 50;

    sprintf(test_str.data.data, "Hello world!");
    test_str.data.size = strlen(test_str.data.data);

    test_msg.data = 0;
	while(1){
        pose.x     = logGetFloat(pitchid);
        pose.y     = logGetFloat(rollid);
        pose.z     = logGetFloat(yawid);
        odom.x     = logGetFloat(Xid);
        odom.y     = logGetFloat(Yid);
        odom.z     = logGetFloat(Zid);
        test_msg.data++;

        RCSOFTCHECK(rcl_publish( &test_publisher, (const void *) &test_msg, NULL));

        RCSOFTCHECK(rcl_publish( &test_string_publisher, (const void *) &test_str, NULL));

        RCSOFTCHECK(rcl_publish( &publisher_attitude, (const void *) &pose, NULL));

        RCSOFTCHECK(rcl_publish( &publisher_odometry, (const void *) &odom, NULL));

        //RCSOFTCHECK(rcl_publish( &weather_publisher, (const void *) &weather_msg, NULL));

        vTaskDelay(1000/portTICK_RATE_MS);
	}

	RCCHECK(rcl_publisher_fini(&publisher_attitude, &node))
	RCCHECK(rcl_publisher_fini(&publisher_odometry, &node))
    RCCHECK(rcl_publisher_fini(&test_publisher, &node))
	RCCHECK(rcl_publisher_fini(&test_string_publisher, &node))

	RCCHECK(rcl_node_fini(&node))

    vTaskSuspend( NULL );
}
