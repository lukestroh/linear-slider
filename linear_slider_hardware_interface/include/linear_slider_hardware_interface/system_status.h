#ifndef __SYSTEM_STATUS_H__
#define __SYSTEM_STATUS_H__

namespace slidersystem{
    enum SystemStatus {
        SYSTEM_OK,
        SYSTEM_STANDBY,
        SYSTEM_CALIBRATING,
        E_STOP,
        NEG_LIM,
        POS_LIM,
    };
}

#endif // __SYSTEM_STATUS_H__