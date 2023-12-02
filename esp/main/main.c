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
#include <std_msgs/msg/int16.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "motores.h"

#define STRING_BUFFER_LEN 50
#define LED GPIO_NUM_2
#define ULTRASONIDO 1
#if ULTRASONIDO
	#include "ultrasonido.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// [ -------------------- VARIABLES DE SUSCRIPTORES Y PUBLISHERS --------------------]
rcl_subscription_t led_subscriber;
std_msgs__msg__Int16 message;

rcl_subscription_t coord_subscriber;
geometry_msgs__msg__Vector3 msgCoord;

#if ULTRASONIDO
	rcl_publisher_t ultraSonido_publisher;
	std_msgs__msg__Int16 msgUltrasonido;

	rcl_subscription_t ultraSonido_subscriber;
	std_msgs__msg__Int16 msgInUS;
#endif

// [ -------------------- VARIABLES GLOBALES --------------------]
uint8_t canMove = 1; //Booleano para habilitar las ruedas
double lastHit = 0;


#if ULTRASONIDO
//Para calcular el tiempo
double dwalltime(){
        double sec;
        struct timeval tv;

        gettimeofday(&tv,NULL);
        sec = tv.tv_sec + tv.tv_usec/1000000.0;
        return sec;
}

#endif

// [ -------------------- FUNCIONES DE SUSCRIPTORES --------------------]

/* Gestor de Subscripción al Tópico /microROS/led
   Se recibe un int_32
   Si data = 0 => Apagado
   Si data = 1 => Prendido
*/
void led_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;

	uint8_t nextState = msg->data;
	if ((nextState >= 0) && (nextState <= 1)){
		gpio_set_level(LED, nextState);
	}
}

/* Gestor de Subscripción al Tópico /microROS/coord
   Se reciben 3 float
   x: coordenada x de joystick
   y: coordenada y de joystick
*/
void coord_subscription_callback(const void * msgin)
{
	if (canMove){
		const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
		printf("(%.2f, %.2f)\n", msg->x, msg->y);
		SetIzqWc(msg->x);
		SetDerWc(msg->y);
	}
	
}
#if ULTRASONIDO
/* Gestor de Subscripción al Tópico /microROS/ultraSonido
   Se reciben un Int16
   Representa la distancia en cm
*/
void ultraSonido_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
	if (msg->data < 10){ //10 cm
		printf("Choque");
		canMove = 0;
		lastHit = dwalltime();
		SetIzqWc(-1);
		SetDerWc(-1);
	}
}
#endif

// [ -------------------- FUNCIONES DE PUBLISHERS --------------------]
#if ULTRASONIDO
void ultrasonido_callback(rcl_timer_t * timer, int64_t last_call_time){
	initUltrasonido();
	static int ultDist = 0;
	(void) last_call_time;
	if (timer != NULL) {
		ultDist = sensarUltrasonido();
		if (ultDist > 0){
			msgUltrasonido.data = ultDist;
			RCSOFTCHECK(rcl_publish(&ultraSonido_publisher, &msgUltrasonido, NULL));
		}
		if ((!canMove) && (dwalltime() - lastHit) >= 0.5)
		{
			canMove = 1;
			SetIzqWc(0);
			SetDerWc(0);
		}
	}
}
#endif


// [ -------------------- FUNCIONES --------------------]
void configurar_GPIO(){
	gpio_set_direction(LED, GPIO_MODE_OUTPUT); 
	initMotorPins();
}

// [ -------------------- TASK DE MICROROS --------------------]
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

	// Create a best effort ping subscriber
	RCCHECK(rclc_subscription_init_default(&led_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/microROS/led"));
	RCCHECK(rclc_subscription_init_default(&coord_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/microROS/coord"));
	// Subscriptor UltraSonido
	#if ULTRASONIDO


	RCCHECK(rclc_subscription_init_default(&ultraSonido_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/microRos/ultraSonido"));

	// Publicador UltraSonido
	RCCHECK(rclc_publisher_init_default(
		&ultraSonido_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
		"/microRos/ultraSonido"));

	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 200;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		ultrasonido_callback));
	#endif
	

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	RCCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &message,
		&led_subscription_callback, ON_NEW_DATA));
		
	RCCHECK(rclc_executor_add_subscription(&executor, &coord_subscriber, &msgCoord,
		&coord_subscription_callback, ON_NEW_DATA));

	#if ULTRASONIDO
	RCCHECK(rclc_executor_add_subscription(&executor, &ultraSonido_subscriber, &msgInUS,
		&ultraSonido_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	#endif

	configurar_GPIO();
	
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

	// Free resources
	#if ULTRASONIDO
		RCCHECK(rcl_subscription_fini(&ultraSonido_subscriber, &node));
	#endif
	RCCHECK(rcl_subscription_fini(&coord_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&led_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}

// [ -------------------- PROGRAMA PRINCIPAL --------------------]
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
