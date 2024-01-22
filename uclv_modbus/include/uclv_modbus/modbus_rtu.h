#ifndef UCLV_MODBUS__MODBUS_RTU_H
#define UCLV_MODBUS__MODBUS_RTU_H

#include "uclv_modbus/usb.h"
#include "uclv_modbus/ethernet.h"
#include "uclv_modbus/crc_calc.h"
#include "uclv_modbus/modbus.h"

namespace uclv {
    /**
     * @brief Class for handling Modbus RTU communication protocol.
     */
    class ModbusRTU : public Modbus
    {
    public:
        /**
         * @brief Constructor for ModbusRTU over USB class.
         * @param device_path Path to the Modbus RTU device.
         * @param slave_id The slave ID of the Modbus device.
         */
        ModbusRTU(const char* device_path, uint8_t slave_id);

        /**
         * @brief Constructor for ModbusRTU over Ethernet class.
         * @param ip_address The IP address of the Modbus RTU device.
         * @param port The port number of the Modbus RTU device.
         * @param slave_id The slave ID of the Modbus device.
        */
        ModbusRTU(const char* ip_address, int port, uint8_t slave_id);

        /**
         * @brief Destructor for ModbusRTU class.
         */
        ~ModbusRTU();

    protected:
        /**
         * @brief Send the Modbus reading request.
         * @param request The request to send.
         * @param request_length The length of the request.
         * @param response_buffer The buffer to store the response in (only registers values).
         * @param response_length The length of the response (only registers values).
         */ 
        void sendRequest(uint8_t *request, uint16_t request_length, uint8_t *response_buffer, uint16_t response_length) override;
    };
} // namespace uclv

#endif // UCLV_MODBUS__MODBUS_RTU_H
