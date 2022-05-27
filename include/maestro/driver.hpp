/// \file maestro/driver.hpp
/// \brief Defines the maestro::driver class.
#ifndef MAESTRO___DRIVER_H
#define MAESTRO___DRIVER_H

#include <vector>
#include <string>

/// \brief Contains all code for the Pololu Maestro.
namespace maestro {

/// \brief A base driver class for the Pololu Maestro series of servo controllers.
class driver
{
public:
    // ENUMERATIONS
    /// \brief An enumeration of Maestro errors.
    enum class error_type
    {
        SERIAL_SIGNAL_ERROR = 0,                ///< Occurs when there is a baud rate mismatch.
        SERIAL_OVERRUN_ERROR = 2,               ///< The Maestro's internal UART buffer has overrun.
        SERIAL_BUFFER_FULL = 4,                 ///< The Maestro's RX buffer has overrun.
        SERIAL_CRC_ERROR = 8,                   ///< CRC mismatch.
        SERIAL_PROTOCOL_ERROR = 16,             ///< Incorrectly formatted data received.
        SERIAL_TIMEOUT = 32,                    ///< Serial timeout period has elapsed.
        SCRIPT_STACK_ERROR = 64,                ///< User script stack underflow or overflow.
        SCRIPT_CALL_STACK_ERROR = 128,          ///< User script call stack underflow or overflow.
        SCRIPT_PROGRAM_COUNTER_ERROR = 256      ///< User script caused program counter to go out of bounds.
    };

    // CONSTRUCTORS
    /// \brief Creates a new driver instance.
    /// \param device_number The device number of the Maestro to communicate with.
    /// \param crc_enabled A flag indicating if the Maestro is configured for CRC mode.
    driver(uint8_t device_number, bool crc_enabled = false);
    ~driver();

    // CONTROL
    /// \brief Starts the driver.
    /// \param serial_port The serial port to open.
    /// \param baud_rate The baud rate to use.
    /// \exception std::runtime_error if the driver fails to start.
    void start(const std::string& serial_port, uint32_t baud_rate);
    /// \brief Stops the driver.
    void stop();

    // SET METHODS
    /// \brief Sets the target position of a specified channel.
    /// \param channel The channel to set the target for.
    /// \param target The target to set, in units of pulse width quarter microseconds.
    void set_target(uint8_t channel, uint16_t target);
    /// \brief Sets targets for multiple channels at once.
    /// \param targets A vector of pairs including each channel and the associated target position,
    /// in units of pulse width quarter microseconds.
    /// \note This command is not compatible with the Micro Maestro.
    void set_target(std::vector<std::pair<uint8_t, uint16_t>> targets);
    /// \brief Limits the speed at which a servo can travel to reach a target.
    /// \param channel The channel to set the speed for.
    /// \param speed The speed to set, in units of pulse width quarter microseconds per 10 milliseconds.
    void set_speed(uint8_t channel, uint16_t speed);
    /// \brief Limits the acceleration at which a servo can travel to reach a target.
    /// \param channel The channel to set the acceleration for.
    /// \param acceleration The acceleration to set, in units of pulse width quarter microseconds per 10 milliseconds per 80 milliseconds.
    void set_acceleration(uint8_t channel, uint16_t acceleration);
    /// \brief Configures the pulse width modulation shape for all channels.
    /// \param on_time The period of the logic high in the duty cycle, in units of 1/48 microseconds.
    /// \param period The overall period of the duty cycle, in units of 1/48 microseconds.
    /// \note This command is not compatible with the Micro Maestro.
    void set_pwm(uint16_t on_time, uint16_t period);
    /// \brief Sends all channels to their home positions.
    void go_home();

    // GET METHODS
    /// \brief Reads the current pulse width of a channel.
    /// \param channel The channel to read the pulse width for.
    /// \returns The current pulse width of the channel, in units of pulse width quarter microseconds.
    uint16_t get_position(uint8_t channel);
    /// \brief Reads if any channels are currently in motion.
    /// \returns TRUE if one or more channels is still in motion, otherwise FALSE.
    /// \details This only functions if the channels are being limited by a speed or acceleration.
    /// \note This command is not compatible with the Micro Maestro.
    bool get_moving_state();
    /// \brief Reads and clears the error register from the Maestro.
    /// \returns The list of all errors as a bit-encoded field.
    uint16_t get_errors();

protected:
    // SERIAL
    /// \brief Opens the serial port for communicating with the Maestro.
    /// \param serial_port The serial port to open.
    /// \param baud_rate The baud rate to use.
    /// \exception std::runtime_error if the port fails to open.
    virtual void open_serial(const std::string& serial_port, uint32_t baud_rate) = 0;
    /// \brief Closes the serial port.
    virtual void close_serial() = 0;
    /// \brief Transmits data to the Maestro through the serial port.
    /// \param data The data byte array to transmit.
    /// \param length The length of the data byte array.
    /// \returns TRUE if the requested bytes were transmitted, otherwise FALSE.
    virtual bool tx(uint8_t* data, uint32_t length) = 0;
    /// \brief Receives data from the Maestro through the serial port.
    /// \param data The buffer to store the data in.
    /// \param length The length of data to read.
    /// \returns TRUE if the requested length was received, otherwise FALSE.
    virtual bool rx(uint8_t* data, uint32_t length) = 0;

private:
    // VARIABLES
    /// \brief The device number of the Maestro to communicate with.
    const uint8_t m_device_number;
    /// \brief Indicates if the Maestro is operating in CRC checking mode.
    const bool m_crc_enabled;

    // SERIAL
    /// \brief Transmits a command over serial using the Pololu protocol.
    /// \param command The command ID.
    /// \param data The serialized data to send.
    /// \param data_length The length of the serialized data to send.
    void tx(uint8_t command, uint8_t* data, uint32_t data_length);
    /// \brief Calculates the CRC-7 checksum of a packet.
    /// \param packet The message packet to calculate the CRC for.
    /// \param length The length of the message packet, in bytes.
    /// \returns The CRC-7 checksum of the packet.
    static uint8_t checksum(uint8_t* packet, uint32_t length);

    // SERIALIZATION
    /// \brief Defines if the host system is little endian.
    bool m_little_endian;
    /// \brief Serializes a uint16_t into a byte array.
    /// \param value The value to serialize.
    /// \param byte_array The byte array to serialize to.
    /// \param position The position in the byte array to serialize to.
    void serialize(uint16_t value, uint8_t* byte_array, uint32_t position);
    /// \brief Deserializes a uint16_t from a byte array.
    /// \param byte_array The byte array to deserialize from.
    /// \param position The position in the byte array to deserialize from.
    /// \returns The deserialized value.
    uint16_t deserialize(uint8_t* byte_array, uint32_t position);
};

}

#endif