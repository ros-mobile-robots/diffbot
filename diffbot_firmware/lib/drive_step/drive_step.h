#ifndef DIFFBOT_DRIVE_STEP_H
#define DIFFBOT_DRIVE_STEP_H

#include <stdint.h>

#include "command_timeout.h"

namespace diffbot {

    /** \brief The command timeout and the control step of loop(), in this order.
     *
     * When the wheel commands become too old, it stops the motors first (eStop()) and logs once
     * after that, because the log message can wait for USB. The control step reads and publishes
     * the wheel states on schedule, and drives the motors (write()) only while the commands are
     * new. A template, so that the unit tests can check the order with a fake controller.
     */
    template <typename TBaseController, typename TLogTimeout>
    void driveStep(TBaseController& base_controller, CommandTimeout& command_timeout,
                   uint32_t now_ms, uint32_t control_period_ms, TLogTimeout log_timeout)
    {
        if (command_timeout.update(now_ms, base_controller.lastUpdateTime().command_received,
                                   base_controller.commandCount()))
        {
            base_controller.eStop();
            log_timeout();
        }

        if (now_ms - base_controller.lastUpdateTime().control >= control_period_ms)
        {
            base_controller.read();
            if (!command_timeout.timedOut())
            {
                base_controller.write();
            }
            base_controller.lastUpdateTime().control = now_ms;
        }
    }
}

#endif // DIFFBOT_DRIVE_STEP_H
