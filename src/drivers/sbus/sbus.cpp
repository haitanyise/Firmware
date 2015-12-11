/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file sbus.cpp
 * Driver for the SBUS on a serial port
 */
#include <drivers/device/device.h>
#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <poll.h>
#include <termios.h>
#include <drivers/drv_rc_input.h>
#include <string.h>
extern "C"{
#include "px4io.h"
}
#include "protocol.h"


#define SBUS0_DEVICE_PATH	"/dev/sbus0"
#define SBUS_DEFAULT_UART_PORT "/dev/ttyS3"

#define TIMEOUT  500
#define RATE_MEASUREMENT_PERIOD 5000000
#define SBUS_WAIT_BEFORE_READ  5
#define SBUS_PACKET_TIMEOUT 1

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2
#define SBUS1_FRAME_DELAY	14000

//static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;
//static hrt_abstime last_txframe_time = 0;
#define IO_POLL_INTERVAL 20000
/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};


class SBUS : public device::CDev
{
public:
	SBUS(const char *uart_path);
	virtual ~SBUS();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int					_serial_fd;					///< serial interface to SBUS
	char				_port[20];					///< device / serial port path
	volatile int		_task;						///< worker task
	bool				_fake_sbus;					///< fake sbus output

	unsigned 			_partial_frame_count;
	unsigned 			_sbus_frame_drops;
	uint8_t				_frame[SBUS_FRAME_SIZE];
	uint16_t 			_rssi;
	uint64_t			_rc_last_valid;				///< last valid timestamp
	orb_advert_t 		_to_input_rc;				///< rc inputs from io
	//perf_counter_t		_perf_update;
	//bool                sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_channels);
	bool				sbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop,
		    							uint16_t max_values);
	int 				receive(const unsigned timeout);
	int					sbus_handle_data(uint8_t *data,uint8_t len);
	int 				io_publish_raw_rc();
	int					io_get_raw_rc_input(rc_input_values &input_rc);

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);


	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sbus_main(int argc, char *argv[]);

namespace
{

SBUS *g_dev = nullptr;

}


SBUS::SBUS(const char *uart_path) :
	CDev("SBUS", SBUS0_DEVICE_PATH),
	_task_should_exit(false),
	_partial_frame_count(0),
	_sbus_frame_drops(0),
	_rssi(0),
	_rc_last_valid(0),
	_to_input_rc(0)
	//_perf_update(perf_alloc(PC_ELAPSED, "sbus update"))
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	_debug_enabled = true;
}

SBUS::~SBUS()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

	g_dev = nullptr;

}

int
SBUS::io_get_raw_rc_input(rc_input_values &input_rc)
{

	uint32_t channel_count;
	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = RC_INPUT_SOURCE_UNKNOWN;

	channel_count = r_page_raw_rc_input[PX4IO_P_RAW_RC_COUNT];

	/* limit the channel count */
	if (channel_count > RC_INPUT_MAX_CHANNELS) {
		channel_count = RC_INPUT_MAX_CHANNELS;
	}

	input_rc.timestamp_publication = hrt_absolute_time();

	input_rc.rc_ppm_frame_length = r_page_raw_rc_input[PX4IO_P_RAW_RC_DATA];

	input_rc.rc_failsafe = (r_page_raw_rc_input[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);

	input_rc.rc_lost = !(r_page_raw_rc_input[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);

	input_rc.rc_lost_frame_count = r_page_raw_rc_input[PX4IO_P_RAW_LOST_FRAME_COUNT];

	input_rc.rc_total_frame_count = r_page_raw_rc_input[PX4IO_P_RAW_FRAME_COUNT];

	input_rc.channel_count = channel_count;


	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp_publication;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = r_page_raw_rc_input[PX4IO_P_RAW_RC_BASE + i];
		//printf("%d:%d ",i,input_rc.values[i]);
	}
	//printf("\r\n");
	/* get RSSI from input channel */
	input_rc.rssi = _rssi;
	return OK;
}
int
SBUS::io_publish_raw_rc()
{
	/* fetch values from IO */
	rc_input_values	rc_val;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK)
		return ret;

	/* sort out the source of the values */
	if (r_status_flags  & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_PPM;

	} else if (r_status_flags & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = RC_INPUT_SOURCE_PX4IO_SBUS;

	} else {
		rc_val.input_source = RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}
	//printf("to input rc:%d\r\n",_to_input_rc);
	/* lazily advertise on first publication */
	if (_to_input_rc == 0) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);

	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);
	}

	return OK;
}

int
SBUS::sbus_handle_data(uint8_t *data,uint8_t len)
{
	uint8_t frist = 0;
	hrt_abstime now = hrt_absolute_time();
	bool sbus_failsafe = false, sbus_frame_drop = false;

	for(int i=0;i<len;i++)
	{
		if(data[i] == 0x0F){
			frist = i;
			if((frist + 25 <= len) && (data[frist + 24] == 0x00)){

				for(uint8_t j=0;j<25;j++)
					_frame[j] = data[frist + j];

				i +=24;
				bool sbus_updated = sbus_decode(now, r_raw_rc_values, &r_raw_rc_count, &sbus_failsafe, &sbus_frame_drop, PX4IO_RC_INPUT_CHANNELS);
				if(sbus_updated){

					r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_SBUS;

					unsigned sbus_rssi = RC_INPUT_RSSI_MAX;

					if (sbus_frame_drop) {
						r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FRAME_DROP;
						sbus_rssi = RC_INPUT_RSSI_MAX / 2;

					} else {
						r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FRAME_DROP);
					}

					if (sbus_failsafe) {
						r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FAILSAFE;

					} else {
						r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
					}
					_rssi = sbus_rssi;
					io_publish_raw_rc();
				}
			}
		}
	}
	return true;
}

bool
SBUS::sbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop,
	    uint16_t max_values)
{
	/* check frame boundary markers to avoid out-of-sync cases */
	if ((_frame[0] != 0x0f)) {
		_sbus_frame_drops++;
		return false;
	}

	switch (_frame[24]) {
	case 0x00:
		/* this is S.BUS 1 */
		break;

	case 0x03:
		/* S.BUS 2 SLOT0: RX battery and external voltage */
		break;

	case 0x83:
		/* S.BUS 2 SLOT1 */
		break;

	case 0x43:
	case 0xC3:
	case 0x23:
	case 0xA3:
	case 0x63:
	case 0xE3:
		break;

	default:
		/* we expect one of the bits above, but there are some we don't know yet */
		break;
	}

	/* we have received something we think is a frame */
	last_frame_time = frame_time;

	unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
			     SBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	for (unsigned channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = _frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}
		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	}

	/* decode switch channels if data fields are wide enough */
	if (PX4IO_RC_INPUT_CHANNELS > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = (_frame[SBUS_FLAGS_BYTE] & (1 << 0)) * 1000 + 998;
		/* channel 18 (index 17) */
		values[17] = (_frame[SBUS_FLAGS_BYTE] & (1 << 1)) * 1000 + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	/* decode and handle failsafe and frame-lost flags */
	if (_frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		*sbus_failsafe = true;
		*sbus_frame_drop = true;

	} else if (_frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 *
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
		 * condition as fail-safe greatly reduces the reliability and range of the radio link,
		 * e.g. by prematurely issueing return-to-launch!!! */

		*sbus_failsafe = false;
		*sbus_frame_drop = true;

	} else {
		*sbus_failsafe = false;
		*sbus_frame_drop = false;
	}

	return true;
}

int
SBUS::receive(const unsigned timeout)
{
	uint8_t buf[128];
	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();
	ssize_t count = 0;
	int handled = 0;

	while (true) {
		bool ready_to_return = false;
		/* poll for new data, wait for only SBUS_PACKET_TIMEOUT (3ms) if something already received */
		int ret =::poll(fds, sizeof(fds) / sizeof(fds[0]),timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			warnx("Sbus poll() err");
			return -1;

		} else if (ret == 0) {
			/* return success after short delay after receiving a packet or timeout after long delay */
			if (ready_to_return) {
				ready_to_return = false;
				return handled;

			} else {
				return -1;
			}

		} else if (ret > 0) {
			/* if we have new data from SBUS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device. But don't read immediately
				 * by 1-2 bytes, wait for some more data to save expensive read() calls.
				 * If more bytes are available, we'll go back to poll() again.
				 */
				usleep(SBUS_WAIT_BEFORE_READ * 1000);
				count = ::read(_serial_fd, buf, 128);
				//printf("count:%d\r\n",count);
				if(count >= 25){
					handled = true;
					ready_to_return = true;
					sbus_handle_data(buf,count);
					return handled;
				}
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < hrt_absolute_time()) {
			return -1;
		}
	}
}

int
SBUS::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* start the GPS driver worker task */
	_task = task_spawn_cmd("sbus", SCHED_DEFAULT,
				SCHED_PRIORITY_SLOW_DRIVER, 1500, (main_t)&SBUS::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

int
SBUS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {
	/*
	case SENSORIOCRESET:
		cmd_reset();
		break;
	*/
	default:
		warnx("sbus ioctl");
		/* give it to parent if no one wants it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	unlock();

	return ret;
}

void
SBUS::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
SBUS::task_main()
{
	struct termios t;
	int termios_state;

	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);// );

	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	/* 100000bps, even parity, two stop bits */
	tcgetattr(_serial_fd, &t);
	//t.c_oflag &= ~ONLCR;
	t.c_cflag |= (CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&t, 100000)) < 0) {
		debug("ERR: %d (cfsetispeed)\n", termios_state);
		_exit(1);
	}

	if ((termios_state = cfsetospeed(&t, 100000)) < 0) {
		debug("ERR: %d (cfsetospeed)\n", termios_state);
		_exit(1);
	}
	tcsetattr(_serial_fd, TCSANOW, &t);
	log("S.bus Ready.");

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {
		//printf("sbus while....\r\n");
		receive(10);
		usleep(10000);
	}

	warnx("exiting");
	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}




void
SBUS::print_info()
{
	warnx("print info");
	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace sbus
{

SBUS	*g_dev = nullptr;

void	start(const char *uart_path);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new SBUS(uart_path);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SBUS0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "open: %s\n", SBUS0_DEVICE_PATH);
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SBUS0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	//if (ioctl(fd, SENSORIOCRESET, 0) < 0)
	//	err(1, "reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "not running");

	g_dev->print_info();

	exit(0);
}

} // namespace


int
sbus_main(int argc, char *argv[])
{

	/* set to default */
	const char *device_name = SBUS_DEFAULT_UART_PORT;
	//bool fake_sbus = false;
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
				/* Detect fake gps option */
		sbus::start(device_name);
	}

	if (!strcmp(argv[1], "stop"))
		sbus::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		sbus::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		sbus::reset();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		sbus::info();

	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'\n [-d /dev/ttyS0-n][-f (for enabling fake)][-s (to enable sat info)]");
}
