#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h" // New oneshot API

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif
#define LED_BUILTIN 33
#define JOYSTICK_BUTTON GPIO_NUM_15 // Use GPIO_NUM_* for clarity
#define JOYSTICK_X ADC_CHANNEL_4    // ADC1_CHANNEL_4 is deprecated; use ADC_CHANNEL_*
#define JOYSTICK_Y ADC_CHANNEL_5    // Same for Y

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;

// Global ADC handle for oneshot mode
adc_oneshot_unit_handle_t adc_handle;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        int raw_x = 0;
        int raw_y = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, JOYSTICK_X, &raw_x));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, JOYSTICK_Y, &raw_y));

        bool joystick_button_state = gpio_get_level(JOYSTICK_BUTTON);

        float right_limit = -0.5f;
        float left_limit = 0.5f;

        float adc1raw = (float)raw_x / 4095.0f; // Normalize 0-1 (12-bit max 4095)
        float adc2raw = (float)raw_y - 2048.0f; // Center Y
        adc2raw /= 1024.0f;                     // Normalize ~ -2 to 2

        if (joystick_button_state == 1) // Forward
        {
            if (adc2raw >= 0) // Left
            {
                if (adc2raw < left_limit)
                {
                    msg.linear.x = adc1raw;
                    msg.angular.z = 0;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
                else
                {
                    adc2raw /= 2.0f;
                    msg.linear.x = adc1raw;
                    msg.angular.z = adc2raw;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
            }
            else if (adc2raw < 0) // Right
            {
                if (adc2raw > right_limit)
                {
                    msg.linear.x = adc1raw;
                    msg.angular.z = 0;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
                else
                {
                    adc2raw /= 2.0f;
                    msg.linear.x = adc1raw;
                    msg.angular.z = adc2raw;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
            }
        }
        else if (joystick_button_state == 0) // Backward
        {

            adc1raw *= -1.0f; // Negate for reverse

            if (adc2raw >= 0) // Left
            {
                if (adc2raw < left_limit)
                {
                    msg.linear.x = adc1raw;
                    msg.angular.z = 0;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
                else
                {
                    adc2raw /= 2.0f;
                    msg.linear.x = adc1raw;
                    msg.angular.z = adc2raw;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
            }
            else if (adc2raw < 0) // Right
            {
                if (adc2raw > right_limit)
                {
                    msg.linear.x = adc1raw;
                    msg.angular.z = 0;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
                else
                {
                    adc2raw /= 2.0f;
                    msg.linear.x = adc1raw;
                    msg.angular.z = adc2raw;
                    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                }
            }
        }

        vTaskDelay(1);
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "teleop_joystick_controller", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "turtle1/cmd_vel"));

    // create timer
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true // Add this: autostart the timer
        ));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // GPIO setup for button
    gpio_set_direction(JOYSTICK_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOYSTICK_BUTTON, GPIO_PULLUP_ONLY);
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_BUILTIN, 0); // Set LED off initially
    // ADC oneshot initialization
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,    // Updated from deprecated DB_11
        .bitwidth = ADC_BITWIDTH_12, // Equivalent to legacy WIDTH_BIT_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_X, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_Y, &chan_cfg));

    // Optional: Cleanup ADC when done (e.g., in a shutdown function)
    // ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));

    // Start Micro-ROS task
    xTaskCreate(micro_ros_task, "uros_task", 4096, NULL, 5, NULL);
}