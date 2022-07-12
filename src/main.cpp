/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"

#include "Arduino.h"
#include "Wire.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

// Adapted from https://github.com/sparkfun/Qwiic_Dual_Encoder_Reader_Py/blob/main/qwiic_dual_encoder_reader.py

typedef enum
{
  SparkFunEncoder_Cmd_ID = 0x00,
  SparkFunEncoder_Cmd_STATUS = 0x01, // 2 - button clicked, 1 - button pressed, 0 - encoder moved
  SparkFunEncoder_Cmd_VERSION = 0x02,
  SparkFunEncoder_Cmd_ENABLE_INTS = 0x04, // #1 - button interrupt, 0 - encoder interrupt
  SparkFunEncoder_Cmd_COUNT1 = 0x05,
  SparkFunEncoder_Cmd_DIFFERENCE1 = 0x07,
  SparkFunEncoder_Cmd_COUNT2 = 0x09,
  SparkFunEncoder_Cmd_DIFFERENCE2 = 0x0B,
  SparkFunEncoder_Cmd_LAST_ENCODER_EVENT = 0x0D, // Millis since last movement of encoder
  SparkFunEncoder_Cmd_TURN_INT_TIMEOUT = 0x0F,
  SparkFunEncoder_Cmd_CHANGE_ADDRESS = 0x11,
  SparkFunEncoder_Cmd_LIMIT = 0x12
} SparkFunEncoder_Cmd;

class I2CPublisher : public rclcpp::Node
{
  public:
    I2CPublisher()
    : Node("ros_qwiic_dual_encoder")
    , _id(0)
    {
    }

    void initialize()
    {
      get_parameter_or<uint8_t>("i2c_address", _id, 0x73); 
      get_parameter_or<std::string>("left_wheel_name", _leftName, "left"); 
      get_parameter_or<std::string>("right_wheel_name", _rightName, "right"); 
      get_parameter_or<double>("poll", _poll, 5.0);
      

      Wire.begin();
      Wire.setAddressSize(1); 

      uint16_t version = readWord(SparkFunEncoder_Cmd_VERSION);
      RCLCPP_INFO(rclcpp::get_logger("ros_qwiic_dual_encoder"), "Talking to encoder version [0x%02x]", version);

      _timer = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(_poll), 
        std::bind(&I2CPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      rclcpp::Time now = this->get_clock()->now();

      uint32_t count1 = readWord(SparkFunEncoder_Cmd_COUNT1);
      if (count1 > 32767)
      {
        // handle wrap around
        count1 -= 65536;
      }

      uint32_t count2 = readWord(SparkFunEncoder_Cmd_COUNT2);
      if (count2 > 32767)
      {
        // handle wrap around
        count2 -= 65536;
      }

      // uint32_t diff1 = readWord(SparkFunEncoder_Cmd_DIFFERENCE1);
      // uint32_t diff2 = readWord(SparkFunEncoder_Cmd_DIFFERENCE2);

      // clear difference?
      // writeWord(SparkFunEncoder_Cmd_DIFFERENCE1, 0);
      //writeWord(SparkFunEncoder_Cmd_DIFFERENCE2, 0);

      // TODO: https://knowyourmeme.com/memes/cmon-do-something
    }

    uint16_t readWord(SparkFunEncoder_Cmd cmd)
    {
      Wire.beginTransmission(_id);
      Wire.write(static_cast<uint8_t>(cmd));
      Wire.endTransmission();

      uint32_t readWord = Wire.requestFrom(_id, sizeof(uint16_t));

      return static_cast<uint16_t>(readWord & 0xFFFF);
    }

    void writeWord(SparkFunEncoder_Cmd cmd, uint16_t value)
    {
      Wire.beginTransmission(_id);
      Wire.write(static_cast<uint8_t>(cmd));
      Wire.write(highByte(value));
      Wire.write(lowByte(value));
      Wire.endTransmission();
    }

    rclcpp::TimerBase::SharedPtr _timer;

    uint8_t _id;
    double _poll;
    std::string _leftName;
    std::string _rightName;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("i2c_address");
    node->declare_parameter("left_wheel_name");
    node->declare_parameter("right_wheel_name");
    node->declare_parameter("poll");

    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}