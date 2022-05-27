#ifndef MAESTRO___ASIO_DRIVER_H
#define MAESTRO___ASIO_DRIVER_H

#include <maestro/driver.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>

#include <deque>
#include <thread>
#include <condition_variable>

namespace maestro {

/// \brief A Boost::ASIO driver for the Pololu Maestro series of servo controllers.
class asio_driver
    : public maestro::driver
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new asio_driver instance.
    /// \param device_number The device number of the Maestro to communicate with.
    /// \param crc_enabled A flag indicating if the Maestro is configured for CRC mode.
    asio_driver(uint8_t device_number, bool crc_enabled = false);

private:
    // SERIAL PORT
    /// \brief The ASIO IO context for the serial port.
    boost::asio::io_service m_io_service;
    /// \brief The ASIO serial port.
    boost::asio::serial_port m_serial_port;
    /// \brief The thread for asynchronous ASIO operations.
    std::thread m_serial_thread;
    /// \brief Provides thread protection for starting/stopping the port.
    std::mutex m_serial_mutex;
    /// \brief The worker function for the ASIO thread.
    void thread_serial();
    // OVERRIDES
    void open_serial(const std::string& serial_port, uint32_t baud_rate, double timeout) override;
    void close_serial() override;

    // TX
    bool tx(uint8_t* data, uint32_t length) override;

    // RX
    /// \brief The buffer for asynchronously receiving partial byte arrays into.
    std::array<uint8_t, 32> m_rx_buffer;
    /// \brief The buffer for storing all received bytes.
    std::deque<uint8_t> m_rx_bytes;
    /// \brief Provides notification of when RX bytes are available.
    std::condition_variable m_rx_condition_variable;
    /// \brief Provides thread protection for RX operations.
    std::mutex m_rx_mutex;
    /// \brief The timeout to wait for received bytes.
    std::chrono::duration<double> m_timeout;
    /// \brief Starts an asynchronous read operation on the serial port.
    void async_receive();
    /// \brief Handles data received from asynchronous read operations.
    void async_receive_callback(const boost::system::error_code& error, uint32_t bytes_read);
    // OVERRIDES
    bool rx(uint8_t* data, uint32_t length) override;
};

}

#endif