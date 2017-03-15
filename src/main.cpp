/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <board.hpp>
#include <chip.h>
#include <uavcan_lpc11c24/uavcan_lpc11c24.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/logger.hpp>
#include <uavcan/equipment/air_data/DiffPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <math.h>
/*
 * GCC 4.9 cannot generate a working binary with higher optimization levels, although
 *         rest of the firmware can be compiled with -Os.
 * GCC 4.8 and earlier don't work at all on this firmware.
 */
#if __GNUC__
# pragma GCC optimize 1
#endif


#define DEFAULT_I2C          I2C0
I2C_ID_T i2cDev = DEFAULT_I2C;  /* Currently active I2C device */

/**
 * This function re-defines the standard ::rand(), which is used by the class uavcan::DynamicNodeIDClient.
 * Redefinition is normally not needed, but GCC 4.9 tends to generate broken binaries if it is not redefined.
 */
int rand()
{
    static int x = 1;
    x = x * 48271 % 2147483647;
    return x;
}



namespace
{

static constexpr unsigned NodeMemoryPoolSize = 2800;



uavcan::Node<NodeMemoryPoolSize>& getNode()
{
    static uavcan::Node<NodeMemoryPoolSize> node(uavcan_lpc11c24::CanDriver::instance(),
                                                 uavcan_lpc11c24::SystemClock::instance());
    return node;
}

uavcan::GlobalTimeSyncSlave& getTimeSyncSlave()
{
    static uavcan::GlobalTimeSyncSlave tss(getNode());
    return tss;
}

uavcan::Logger& getLogger()
{
    static uavcan::Logger logger(getNode());
    return logger;
}

void init()
{
    board::resetWatchdog();

    board::setErrorLed(false);
    board::setStatusLed(true);

    /*
     * Configuring the clock - this must be done before the CAN controller is initialized
     */
    uavcan_lpc11c24::clock::init();
    /*
     * Configuring the CAN controller
     */
    std::uint32_t bit_rate = 0;
    while (bit_rate == 0)
    {
        bit_rate = uavcan_lpc11c24::CanDriver::detectBitRate(&board::resetWatchdog);
    }


    if (uavcan_lpc11c24::CanDriver::instance().init(bit_rate) < 0)
    {
        board::die();
    }


    board::resetWatchdog();

    /*
     * Configuring the node
     */
    getNode().setName("org.uavcan.lpc11c24_test");

    uavcan::protocol::SoftwareVersion swver;
    swver.major = FW_VERSION_MAJOR;
    swver.minor = FW_VERSION_MINOR;
    swver.vcs_commit = GIT_HASH;
    swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    getNode().setSoftwareVersion(swver);

    uavcan::protocol::HardwareVersion hwver;
    std::uint8_t uid[board::UniqueIDSize] = {};
    board::readUniqueID(uid);
    std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));
    getNode().setHardwareVersion(hwver);

    board::resetWatchdog();

    /*
     * Starting the node and performing dynamic node ID allocation
     */
    if (getNode().start() < 0)
    {
        board::die();
    }


    getNode().setNodeID(21);

    board::resetWatchdog();

    /*
     * Initializing other libuavcan-related objects
     */
    if (getTimeSyncSlave().start() < 0)
    {
        board::die();
    }

    if (getLogger().init() < 0)
    {
        board::die();
    }

    getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

    board::resetWatchdog();
}
}


int main()
{
    init();
    getNode().setModeOperational();

    uavcan::MonotonicTime prev_log_at;
    uavcan::Publisher<uavcan::equipment::air_data::DiffPressure>          _uavcan_pub_pressure(getNode());
    uavcan::Publisher<uavcan::equipment::air_data::StaticTemperature>           _uavcan_pub_temperature(getNode());
    //start I2C read
    uint8_t cmd = 0;
    _uavcan_pub_pressure.setPriority(6);
    _uavcan_pub_temperature.setPriority(6);
    Chip_I2C_MasterSend(i2cDev,0x28,&cmd,1);

    while (true)
    {
        getNode().spin(uavcan::MonotonicDuration::fromMSec(10));

        uint8_t dat[4] = {0};


        /* Setup I2C parameters to send 4 bytes of data */

        Chip_I2C_MasterRead(i2cDev, 0x28, dat, 4);

        Chip_I2C_MasterSend(i2cDev, 0x28,&cmd,1);

        uint8_t status = (dat[0] & 0xC0) >> 6;
        if (status == 2 || status == 3) {
            continue;
        }

        int16_t dp_raw, dT_raw;
        dp_raw = int16_t((uint16_t(dat[0]) << 8) + uint16_t(dat[1]));
        dp_raw = 0x3FFF & dp_raw;
        dT_raw = int16_t((uint16_t(dat[2]) << 8) + uint16_t(dat[3]));
        dT_raw = (0xFFE0 & dT_raw) >> 5;

        const float P_max = 1.0f;
        const float P_min = - P_max;
        const float PSI_to_Pa = 6894.757f;
        /*
          this equation is an inversion of the equation in the
          pressure transfer function figure on page 4 of the datasheet
          We negate the result so that positive differential pressures
          are generated when the bottom port is used as the static
          port on the pitot and top port is used as the dynamic port
         */
        float diff_press_PSI = -((float(dp_raw) - 0.1f*16383.0f) * (P_max-P_min)/(0.8f*16383.0f) + P_min);

        float press = diff_press_PSI * PSI_to_Pa;
        float temp = ((200.0f * dT_raw) / 2047) - 50;
        //float raw_airspeed = sqrtf(press * 2.0f);
        /*
         * CAN error counter, for debugging purposes
         */


        uavcan::equipment::air_data::StaticTemperature temp_msg;
        uavcan::equipment::air_data::DiffPressure press_msg;
        temp_msg.static_temperature = temp;
        press_msg.differential_pressure = press;
        (void)_uavcan_pub_pressure.broadcast(press_msg);
        (void)_uavcan_pub_temperature.broadcast(temp_msg);

        board::resetWatchdog();
    }
}
