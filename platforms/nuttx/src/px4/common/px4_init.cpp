/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/init.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform/cpuload.h>
#include <uORB/uORB.h>

#include <fcntl.h>

#if defined(CONFIG_I2C)
# include <px4_platform_common/i2c.h>
# include <nuttx/i2c/i2c_master.h>
#endif // CONFIG_I2C

#if defined(CONFIG_SYSTEM_CDCACM)
__BEGIN_DECLS
#include <nuttx/wqueue.h>
#include <builtin/builtin.h>

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);
__END_DECLS

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

static struct work_s usb_serial_work;
static bool vbus_present_prev = false;

enum class UsbAutoStartState {
	disconnected,
	connecting,
	connected,
	disconnecting,
} usb_auto_start_state{UsbAutoStartState::disconnected};

static void mavlink_usb_check(void *arg)
{
	int rescheduled = -1;

	uORB::SubscriptionData<actuator_armed_s> actuator_armed_sub{ORB_ID(actuator_armed)};

	const bool armed = actuator_armed_sub.get().armed;
	const bool vbus_present = (board_read_VBUS_state() == PX4_OK);

	if (!armed) {
		switch (usb_auto_start_state) {
		case UsbAutoStartState::disconnected:
			if (vbus_present && vbus_present_prev) {
				usb_auto_start_state = UsbAutoStartState::connecting;
				rescheduled = work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, NULL, 0);

			} else if (vbus_present && !vbus_present_prev) {
				// check again sooner is USB just connected
				rescheduled = work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, NULL, USEC2TICK(400000));
			}

			break;

		case UsbAutoStartState::connecting:
			sched_lock();

			if (sercon_main(0, NULL) == EXIT_SUCCESS) {
				static const char *mavlink_start_argv[] {"mavlink", "start", "-d", "/dev/ttyACM0", NULL};

				if (exec_builtin("mavlink", (char **)mavlink_start_argv, NULL, 0) > 0) {
					usb_auto_start_state = UsbAutoStartState::connected;

				} else {
					usb_auto_start_state = UsbAutoStartState::disconnecting;
				}

			} else {
				// back to disconnecting
				usb_auto_start_state = UsbAutoStartState::disconnecting;
			}

			sched_unlock();

			break;

		case UsbAutoStartState::connected:
			if (!vbus_present && !vbus_present_prev) {
				sched_lock();
				static const char *mavlink_stop_argv[] {"mavlink", "stop", "-d", "/dev/ttyACM0", NULL};
				exec_builtin("mavlink", (char **)mavlink_stop_argv, NULL, 0);
				sched_unlock();

				usb_auto_start_state = UsbAutoStartState::disconnecting;
			}

			break;

		case UsbAutoStartState::disconnecting:
			// serial disconnect if unused
			serdis_main(0, NULL);
			usb_auto_start_state = UsbAutoStartState::disconnected;

			break;
		}
	}

	vbus_present_prev = vbus_present;

	if (rescheduled != PX4_OK) {
		work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, NULL, USEC2TICK(1000000));
	}
}

#endif // CONFIG_SYSTEM_CDCACM

int px4_platform_init(void)
{

	int ret = px4_console_buffer_init();

	if (ret < 0) {
		return ret;
	}

	// replace stdout with our buffered console
	int fd_buf = open(CONSOLE_BUFFER_DEVICE, O_WRONLY);

	if (fd_buf >= 0) {
		dup2(fd_buf, 1);
		// keep stderr(2) untouched: the buffered console will use it to output to the original console
		close(fd_buf);
	}

	hrt_init();

	param_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif


#if defined(CONFIG_I2C)
	I2CBusIterator i2c_bus_iterator {I2CBusIterator::FilterType::All};

	while (i2c_bus_iterator.next()) {
		i2c_master_s *i2c_dev = px4_i2cbus_initialize(i2c_bus_iterator.bus().bus);

#if defined(CONFIG_I2C_RESET)
		I2C_RESET(i2c_dev);
#endif // CONFIG_I2C_RESET

		// send software reset to all
		uint8_t buf[1] {};
		buf[0] = 0x06; // software reset

		i2c_msg_s msg{};
		msg.frequency = I2C_SPEED_STANDARD;
		msg.addr = 0x00; // general call address
		msg.buffer = &buf[0];
		msg.length = 1;

		I2C_TRANSFER(i2c_dev, &msg, 1);

		px4_i2cbus_uninitialize(i2c_dev);
	}

#endif // CONFIG_I2C


	px4::WorkQueueManagerStart();

	uorb_start();

	px4_log_initialize();

#if defined(CONFIG_SYSTEM_CDCACM)
	work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, NULL, USEC2TICK(100000));
#endif // CONFIG_SYSTEM_CDCACM

	return PX4_OK;
}

int px4_platform_configure(void)
{
	return px4_mft_configure(board_get_manifest());

}
