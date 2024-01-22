#ifndef UCLV_MODBUS__MODBUS_TCP_H
#define UCLV_MODBUS__MODBUS_TCP_H

#include "uclv_modbus/ethernet.h"
#include "uclv_modbus/usb.h"
#include "uclv_modbus/modbus.h"

namespace uclv
{
    /**
     * @brief Class for handling Modbus TCP communication protocol.
     */
    class ModbusTCP : public Modbus
    {
    public:
        /**
         * @brief Constructor for ModbusTCP over Ethernet class.
         * @param ip_address The IP address of the Modbus TCP device.
         * @param port The port number of the Modbus TCP device.
         * @param slave_id The slave ID of the Modbus device.
         */
        ModbusTCP(const char* ip_address, int port, uint8_t slave_id);

        /**
         * @brief Destructor for ModbusTCP class.
         */
        ~ModbusTCP();

    protected:
        /**
         * @brief Send the Modbus reading request.
         * @param request The request to send.
         * @param request_length The length of the request.
         * @param response_buffer The buffer to store the response in (only registers values).
         * @param response_length The length of the response (only registers values).
         */ 
        void sendRequest(uint8_t *request, uint16_t request_length, uint8_t *response_buffer, uint16_t response_length) override;

    private:
        uint16_t transaction_id_{0};

    };
} // namespace uclv

#endif // UCLV_MODBUS__MODBUS_TCP_H