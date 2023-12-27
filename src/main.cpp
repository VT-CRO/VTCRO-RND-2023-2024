// #include "arduino_freertos.h"
// #include "avr/pgmspace.h"
// #include "MotorControl.h"

// static void task1(void*) {
//   while (true) {
//     digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
//     vTaskDelay(pdMS_TO_TICKS(500));

//     digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }

// static void task2(void*) {
//   while (true) {
//     Serial.println("TICK");
//     vTaskDelay(pdMS_TO_TICKS(1'000));

//     Serial.println("TOCK");
//     vTaskDelay(pdMS_TO_TICKS(1'000));
//   }
// }

// FLASHMEM __attribute__((noinline))
// void setup() {
//   // Serial.begin(115'200);
//   // pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
//   // digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);

//   // delay(5'000);

//   // if (CrashReport)  {
//   //   Serial.print(CrashReport);
//   //   Serial.println();
//   //   Serial.flush();
//   // }

//   // Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

//   // // create tasks
//   // xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
//   // xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);

//   // // start the scheduler
//   // Serial.println("setup(): starting scheduler...");
//   // Serial.flush();


   
//   MotorControl m;
//   m.Motor_pin_init();
//   m.Motor_start();
//   // while(1){
//   //   Serial.print("The motors should have started\n");
//   // }
//  // m.Motor_setSpeed(127,127,127,127);
  
//   vTaskStartScheduler();

// }


// void loop() {
//   // put your main code here, to run repeatedly:
  
//   // MotorControl m;
//   // m.Motor_pin_init();
//   // while(1){
//   // m.Motor_start();
//   // delay(1000);
//   // m.stopMove();
// }

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
