#include <maestro/driver.hpp>

#include <stdexcept>

using namespace maestro;

// CONSTRUCTORS
driver::driver(uint8_t device_number, bool crc_enabled)
    : m_device_number(device_number),
      m_crc_enabled(crc_enabled)
{
    // Check endianness of the host.
    uint32_t integer = 1;
    driver::m_little_endian = reinterpret_cast<uint8_t*>(&integer)[0];
}
driver::~driver()
{
    // Stop the driver if it's running.
    driver::stop();
}

// CONTROL
void driver::start(const std::string& serial_port, uint32_t baud_rate, double timeout)
{
    // Open the serial port.
    driver::open_serial(serial_port, baud_rate, timeout);

    // Try to transmit a single 0xAA delimiter to set up automatic baud rate detection.
    uint8_t delimiter = 0xAA;
    if(!driver::tx(&delimiter, 1))
    {
        throw std::runtime_error("error sending delimiter for baud rate detection");
    }
}
void driver::stop()
{
    // Stop the serial port.
    driver::close_serial();
}

// SET METHODS
void driver::set_target(uint8_t channel, uint16_t target)
{
    // Create data array.
    uint8_t data[3];
    // Set channel.
    data[0] = channel;
    // Set target.
    driver::serialize(target, data, 1);

    // Send the message.
    driver::tx(0x04, data, 3);
}
void driver::set_target(std::vector<std::pair<uint8_t, uint16_t>> targets)
{
    // Create data array.
    uint32_t length = static_cast<uint32_t>(targets.size() * 3);
    uint8_t* data = new uint8_t[length];

    // Iterate through the channels.
    for(uint32_t i = 0; i < targets.size(); i++)
    {
        // Get reference to pair.
        std::pair<uint8_t, uint16_t>& current = targets.at(i);
        // Set channel.
        data[i*3] = current.first;
        // Set target.
        driver::serialize(current.second, data, i*3+1);
    }

    // Send the message.
    driver::tx(0x1F, data, length);

    // Delete the data.
    delete [] data;
}
void driver::set_speed(uint8_t channel, uint16_t speed)
{
    // Create data array.
    uint8_t data[3];
    // Set channel.
    data[0] = channel;
    // Set speed.
    driver::serialize(speed, data, 1);

    // Send the message.
    driver::tx(0x07, data, 3);
}
void driver::set_acceleration(uint8_t channel, uint16_t acceleration)
{
    // Create data array.
    uint8_t data[3];
    // Set channel.
    data[0] = channel;
    // Set acceleration
    driver::serialize(acceleration, data, 1);

    // Send the message.
    driver::tx(0x09, data, 3);
}
void driver::set_pwm(uint16_t on_time, uint16_t period)
{
    // Create data array.
    uint8_t data[4];
    // Set on_time.
    driver::serialize(on_time, data, 0);
    // Set period.
    driver::serialize(period, data, 2);

    // Send the message.
    driver::tx(0x0A, data, 4);
}
void driver::go_home()
{
    driver::tx(0x22, nullptr, 0);
}

// GET METHODS
uint16_t driver::get_position(uint8_t channel)
{
    // Create data array.
    uint8_t data[1] = {channel};
    // Send the message.
    driver::tx(0x10, data, 1);

    // Read the response.
    uint8_t response[2];
    if(driver::rx(response, 2))
    {
        // Deserialize response.
        return driver::deserialize(response, 0);
    }
    else
    {
        throw std::runtime_error("response not received");
    }
}
bool driver::get_moving_state()
{
    // Send the message.
    driver::tx(0x13, nullptr, 0);

    // Read the response.
    uint8_t response;
    if(driver::rx(&response, 1))
    {
        // Deserialize response.
        return static_cast<bool>(response);
    }
    else
    {
        throw std::runtime_error("response not received");
    }
}
uint16_t driver::get_errors()
{
    // Send the message.
    driver::tx(0x21, nullptr, 0);

    // Read the response.
    uint8_t response[2];
    if(driver::rx(response, 2))
    {
        // Deserialize response.
        return driver::deserialize(response, 0);
    }
    else
    {
        throw std::runtime_error("response not received");
    }
}

// TRANSMISSION METHODS
void driver::tx(uint8_t command, uint8_t *data, uint32_t data_length)
{
    // Create a new packet to send.
    // Pololu Protocol: Header (1), Device Number (1), Command ID (1), data bytes, Checksum if enabled (1)
    uint32_t packet_length = 3 + data_length + static_cast<uint32_t>(driver::m_crc_enabled);
    uint8_t* packet = new uint8_t[packet_length];

    // Set packet fields.
    packet[0] = 0xAA;
    packet[1] = driver::m_device_number;
    packet[2] = command;
    for(uint32_t i = 0; i < data_length; ++i)
    {
        packet[3+i] = data[i];
    }

    // Add checksum if needed.
    if(driver::m_crc_enabled)
    {
        packet[packet_length-1] = driver::checksum(packet, packet_length - 1);
    }

    // Transmit the bytes via serial.
    if(!driver::tx(packet, packet_length))
    {
        throw std::runtime_error("failed to transmit command");
    }

    // Delete the packet.
    delete [] packet;
}
uint8_t driver::checksum(uint8_t *packet, uint32_t length)
{
    // Calculate the CRC-7 checksum (https://www.pololu.com/docs/0J44/6.7.6)
    const uint8_t polynomial = 0x91;

    uint8_t crc = 0;

    // Iterate over the bytes of the message.
    for(uint32_t i = 0; i < length; i++)
    {
        crc ^= packet[i];
        // Iterate over the bits of the current byte.
        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 1)
            {
                crc ^= polynomial;
            }
            crc >>= 1;
        }
    }

    return crc;
}
void driver::serialize(uint16_t value, uint8_t *buffer, uint32_t position)
{
    // NOTE: Maestro is little endian, and uses 14bit numbers.
    // byte_one = first seven bits
    // byte_two = second seven bits

    // Check endianness.
    if(driver::m_little_endian)
    {
        // Direct copy for little endian.
        buffer[position] = value & 0x007FU;
        buffer[position+1] = (value & 0x3F80U) >> 7;
    }
    else
    {
        // Swap to little endian.
        buffer[position] = (value & 0x3F80U) >> 7;
        buffer[position+1] = value & 0x007FU;
    }
}
uint16_t driver::deserialize(uint8_t *buffer, uint32_t position)
{
    // Create output variable.
    uint16_t output;

    // Check endianness.
    if(driver::m_little_endian)
    {
        // Direct copy for littel endian.
        output = buffer[position];
        output |= static_cast<uint16_t>(buffer[position+1]) << 7;
    }
    else
    {
        // Swap to little endian.
        output = buffer[position+1];
        output |= static_cast<uint16_t>(buffer[position]) << 7;
    }

    return output;
}