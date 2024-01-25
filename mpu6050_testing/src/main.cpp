#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_READABLE_YAWPITCHROLL

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    rosidl_runtime_c__String__assign(&msg.header.frame_id, "plane");
    RCSOFTCHECK(rcl_publish(&imu_pub, &msg, NULL));
  }
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    while (!Serial); 

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "imu_pub", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu"));

    const unsigned int timer_timeout = 1000; // create timer,
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    mpu.initialize();

    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(273.00000);
    mpu.setYGyroOffset(-46.00000);
    mpu.setZGyroOffset(115.00000);
    mpu.setZAccelOffset(1212.00000);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            msg.orientation.x = -q.x;
            msg.orientation.y = -q.y;
            msg.orientation.z = q.z;
            msg.orientation.w = q.w;
            msg.orientation_covariance[0] = -1;
        #endif

        #ifdef OUTPUT_READABLE_ACCELGYRO
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            msg.angular_velocity.x = gx;
            msg.angular_velocity.y = gy;
            msg.angular_velocity.z = gz;
            msg.angular_velocity_covariance[0] = -1;
            msg.linear_acceleration.x = ax;
            msg.linear_acceleration.y = ay;
            msg.linear_acceleration.z = az;
            msg.linear_acceleration_covariance[0] = -1;
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            msg[0].data = ypr[0] * 180/M_PI;
            msg[1].data = ypr[1] * 180/M_PI;
            msg[2].data = ypr[2] * 180/M_PI;
        #endif
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10000)));
}


