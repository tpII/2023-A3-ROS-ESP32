#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#endif

#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <std_msgs/msg/string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "motores.h"

#define STRING_BUFFER_LEN 50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t string_subscriber;
std_msgs__msg__String msgString;

#define LED GPIO_NUM_2

int canMove = 1; //Booleano para habilitar las ruedas
double lastHit = 0;


//Para calcular el tiempo
double dwalltime(){
        double sec;
        struct timeval tv;

        gettimeofday(&tv,NULL);
        sec = tv.tv_sec + tv.tv_usec/1000000.0;
        return sec;
}

// Format: "var-name":value;
void string_subscription_callback(const void* msgin) {
	const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
	printf("String recibido: %s\n", msg->data.data);
	
	// Parse
	// Coord
	char* str = msg->data.data;
	float x = *(float*) &str[0];
	float y = *(float*) &str[4];
	if (canMove) {
		printf("Coord: (%.2f, %.2f)\n", x, y);
		SetIzqWc(x);
		SetDerWc(y);
	}

	// LED
	char led_state = str[8];
	if ((led_state >= 0) && (led_state <= 1)){
		printf("LED state: %d\n", led_state);
		gpio_set_level(LED, led_state);
	}

	// Ultrasound
	int ults = *(int*) &str[9];
	if (ults < 10){ //10 cm
		printf("Choque %d\n", ults);
		canMove = 0;
		lastHit = dwalltime();
		SetIzqWc(-1);
		SetDerWc(-1);
	}

	// Parse
	char* arg;
}

void configurar_GPIO(){
	gpio_set_direction(LED, GPIO_MODE_OUTPUT); 
	initMotorPins();
}


void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

#if 0
	// create init_options
	//RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
#endif

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	//Nodo llamado LED_NODE
	RCCHECK(rclc_node_init_default(&node, "led_node", "", &support));

	RCCHECK(rclc_subscription_init_best_effort(&string_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/string"));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_add_subscription(&executor, &string_subscriber, &msgString,
		&string_subscription_callback, ON_NEW_DATA));

	configurar_GPIO();
	
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_subscription_fini(&string_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}


void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

}
