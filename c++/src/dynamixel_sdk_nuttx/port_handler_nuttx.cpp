/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <px4_log.h>
#include <drivers/drv_gpio.h>

#include "dynamixel_sdk_nuttx/port_handler_nuttx.h"

// TODO: TEMP
#include <errno.h>
#include <px4_log.h>
#include "errno_str.h"

#define LATENCY_TIMER   4  // msec (USB latency timer)

using namespace dynamixel;

PortHandlerNuttx::PortHandlerNuttx(const char *port_name, uint8_t gpio)
  : baudrate_(DEFAULT_BAUDRATE_),
    _serial_port{port_name},
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0),
    gpio_{0}
{
  gpio_ = 1 << (gpio-1);
  is_using_ = false;
  //setPortName(port_name);
}

bool PortHandlerNuttx::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandlerNuttx::closePort()
{
  _serial_port.close();
}

void PortHandlerNuttx::clearPort()
{
  _serial_port.clear();
}

void PortHandlerNuttx::setPortName(const char *port_name)
{
  //strcpy(port_name_, port_name);
  PX4_ERR("setPortName not implemented.");
}

char *PortHandlerNuttx::getPortName()
{
  return _serial_port.get_port_name();
}

// TODO: baud number ??
bool PortHandlerNuttx::setBaudRate(const int baudrate)
{
  // see src/modules/mavlink/mavlink_main.cpp:744 for example

  int baud = getCFlagBaud(baudrate);

  if(baud <= 0)   // custom baudrate
  {
    setupPort(B38400);
    baudrate_ = baudrate;
    return setCustomBaudrate(baudrate);
  }
  else
  {
    return setupPort(baud);
  }
}

int PortHandlerNuttx::getBaudRate()
{
  return baudrate_;
}

int PortHandlerNuttx::getBytesAvailable()
{
  return _serial_port.get_bytes_available();
}

int PortHandlerNuttx::readPort(uint8_t *packet, int length)
{
  // See /src/modules/mavlink/mavlink_receiver.cpp:2130 for example
  // This is now somewhat equal to the example, but is not tested - glenn
  return _serial_port.read(packet, length);
}

int PortHandlerNuttx::writePort(uint8_t *packet, int length)
{
  // See: src/modules/mavlink/mavlink_main.cpp:996 for example
  // This is now equal to the example, and should work, but is not tested - glenn

  // GPIO
  auto counter = MAX_GPIO_ATTEMPTS;
  while (!controlGpio(true) && counter--)
  {
    if (counter <= 0)
    {
      PX4_ERR("Could not set GPIO");
      return -1;
    }
  }

  // UART
  auto res = _serial_port.write(packet, length);

  // GPIO
  counter = MAX_GPIO_ATTEMPTS;
  while (!controlGpio(false) && counter--)
  {
    if (counter <= 0)
    {
      PX4_ERR("Could not clear GPIO");
      return -1;
    }
  }

  return res;
}

void PortHandlerNuttx::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerNuttx::setPacketTimeout(double msec)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandlerNuttx::isPacketTimeout()
{
  if(getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }
  return false;
}

double PortHandlerNuttx::getCurrentTime()
{
	struct timespec tv;
	clock_gettime( CLOCK_REALTIME, &tv);
	return ((double)tv.tv_sec*1000.0 + (double)tv.tv_nsec*0.001*0.001);
}

double PortHandlerNuttx::getTimeSinceStart()
{
  double time;

  time = getCurrentTime() - packet_start_time_;
  if(time < 0.0)
    packet_start_time_ = getCurrentTime();

  return time;
}

bool
PortHandlerNuttx::controlGpio(bool transmit)
{/*
  auto gpio_fd = ::open(PX4FMU_DEVICE_PATH, O_WRONLY);
  if (gpio_fd < 0)
  {
    PX4_ERR("Could not open GPIO gpio_fd.");
    return false;
  }

  auto cmd = GPIO_CLEAR;
  if (transmit)
  {
    cmd = GPIO_SET;
  }

  auto fault = ioctl(gpio_fd, cmd, gpio_);
  if (fault)
  {
    PX4_ERR("Could not change gpio: %d", fault);
    close(gpio_fd);
    return false;
  }
  close(gpio_fd);*/
  return true;
}

bool PortHandlerNuttx::setupPort(int cflag_baud)
{
  // see src/px4/firmware/src/modules/mavlink/mavlink_main.cpp:630 for example
  /*
  auto gpio_fd = ::open(PX4FMU_DEVICE_PATH, O_WRONLY);
  
  if (gpio_fd < 0)
  {
    PX4_ERR("Could not open GPIO gpio_fd.");
    return false;
  }
  else
  {
    PX4_INFO("GPIO filedes: %d", gpio_fd);
  }

  auto fault = ioctl(gpio_fd, GPIO_SET_OUTPUT, gpio_);
  if (fault)
  {
    PX4_ERR("Set output failed: %d", fault);
    close(gpio_fd);
    return false;
  }
  close(gpio_fd);

  if (!controlGpio(false))
  {
    PX4_ERR("Initial gpio receive failed");
    return false;
  }*/

  _serial_port.close();
  _serial_port.open();
  auto res = _serial_port.setup(cflag_baud) == 0;

  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return res;
}

bool PortHandlerNuttx::setCustomBaudrate(int speed)
{
  PX4_ERR("Custom baudrate is not supported.");
  return false;
}

int PortHandlerNuttx::getCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
    default:
      return -1;
  }
}
