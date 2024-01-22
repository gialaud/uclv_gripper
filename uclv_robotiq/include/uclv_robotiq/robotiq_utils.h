#ifndef UCLV_ROBOTIQ__ROBOTIQ_UTILS_H
#define UCLV_ROBOTIQ__ROBOTIQ_UTILS_H

#include <cstdint>

namespace uclv
{
    // a struct to hold status and command for finger
    struct GripperStatus
    {
        // gripper status
        uint8_t gACT{0};
        uint8_t gMOD{0};
        uint8_t gGTO{0};
        uint8_t gSTA{0}; // gripper status: in reset, activation in progress, mode change in progress, active
        uint8_t gOBJ{0}; // motion status
        uint8_t gFLT{0};
    };

    struct FingerStatus
    {
        uint8_t gPR{0};
        uint8_t gPO{0};
        uint8_t gCU{0};
        uint8_t gDT{0}; // motion status, related to object detection
    };

    struct Finger
    {
        uint8_t id;
        // request
        uint8_t rPR{0};
        uint8_t rSP{100};
        uint8_t rFR{100};
        FingerStatus status;
    };

} // namespace uclv
#endif // UCLV_ROBOTIQ__ROBOTIQ_UTILS_H
