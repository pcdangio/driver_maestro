#include <maestro/asio_driver.hpp>

#include <boost/asio/write.hpp>

using namespace maestro;

// CONSTRUCTORS
asio_driver::asio_driver(uint8_t device_number, bool crc_enabled)
    : driver(device_number, crc_enabled),
      m_serial_port(asio_driver::m_io_service)
{}

// SERIAL PORT
void asio_driver::thread_serial()
{
    // Create work guard to keep the IO service running if no work is present.
    boost::asio::io_service::work work_guard(asio_driver::m_io_service);

    // Run the IO service.
    asio_driver::m_io_service.run();
}
void asio_driver::open_serial(const std::string& serial_port, uint32_t baud_rate, double timeout)
{
    // Lock serial for the duration of this function.
    std::lock_guard<std::mutex> lock_guard(asio_driver::m_serial_mutex);

    // Check if the port is already running.
    if(asio_driver::m_serial_port.is_open())
    {
        return;
    }
    
    // Open serial port.
    boost::system::error_code error;
    asio_driver::m_serial_port.open(serial_port, error);
    if(error)
    {
        throw std::runtime_error("failed to open serial port (" + error.message() + ")");
    }

    // Set serial port's baud rate.
    asio_driver::m_serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate), error);
    if(error)
    {
        throw std::runtime_error("failed to set serial port baud rate (" + error.message() + ")");
    }

    // Start the io_context thread.
    asio_driver::m_serial_thread = std::thread(&asio_driver::thread_serial, this);

    // Start asynchronously receiving.
    asio_driver::async_receive();

    // Store timeout.
    asio_driver::m_timeout = std::chrono::duration<double>(timeout);
}
void asio_driver::close_serial()
{
    // Lock serial for the duration of this function.
    std::lock_guard<std::mutex> lock_guard(asio_driver::m_serial_mutex);

    // Check if serial port is running.
    if(!asio_driver::m_serial_port.is_open())
    {
        return;
    }

    // Close the serial port.
    boost::system::error_code error;
    asio_driver::m_serial_port.close(error);
    if(error)
    {
        throw std::runtime_error("failed to close serial port (" + error.message() + ")");
    }

    // Stop the IO service.
    asio_driver::m_io_service.stop();

    // Join the IO service thread.
    if(asio_driver::m_serial_thread.joinable())
    {
        asio_driver::m_serial_thread.join();
    }
}

// TX
bool asio_driver::tx(uint8_t* data, uint32_t length)
{
    // Write the data synchronously to the serial port.
    boost::system::error_code error;
    uint32_t write_count = boost::asio::write(asio_driver::m_serial_port, boost::asio::buffer(data, length), error);

    // Check if transmission was successful.
    return !error && (write_count == length);
}

// RX
void asio_driver::async_receive()
{
    // Start async receive operation.
    asio_driver::m_serial_port.async_read_some(boost::asio::buffer(asio_driver::m_rx_buffer),
                                               std::bind(&asio_driver::async_receive_callback, this, std::placeholders::_1, std::placeholders::_2));
}
void asio_driver::async_receive_callback(const boost::system::error_code& error, uint32_t bytes_read)
{
    // Check if bytes were read.
    if(bytes_read)
    {
        // Lock thread protection for the RX bytes deque.
        asio_driver::m_rx_mutex.lock();

        // Add the new bytes to the deque.
        for(uint32_t i = 0; i < bytes_read; ++i)
        {
            asio_driver::m_rx_bytes.push_back(asio_driver::m_rx_buffer[i]);
        }

        // Unlock thread protection for RX bytes deque.
        asio_driver::m_rx_mutex.unlock();

        // Notify the RX condition variable that new bytes are available.
        asio_driver::m_rx_condition_variable.notify_one();
    }

    // Run the next async receive.
    asio_driver::async_receive();
}
bool asio_driver::rx(uint8_t* data, uint32_t length)
{
    // Create unique lock for RX bytes condition variable.
    std::unique_lock<std::mutex> unique_lock(asio_driver::m_rx_mutex);

    // Wait (with timeout) for RX byte deque to contain at least length bytes.
    // Returns TRUE if this condition is satisified, FALSE if the operation timed out.
    if(!asio_driver::m_rx_condition_variable.wait_for(unique_lock, asio_driver::m_timeout, [this, length](){return asio_driver::m_rx_bytes.size() >= length;}))
    {
        return false;
    }

    // If this point reached, there are enough bytes in the RX byte deque and the mutex is still locked.
    // Move requested number of bytes into data array.
    for(uint32_t i = 0; i < length; ++i)
    {
        data[i] = asio_driver::m_rx_bytes.front();
        asio_driver::m_rx_bytes.pop_front();
    }

    return true;
}