#ifndef ROVER_CONTROLLER_H
#define ROVER_CONTROLLER_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {
typedef uint32_t (*MillisFn)();

class RoverController {
public:
    explicit RoverController(MillisFn now_ms,
                             uint32_t watchdog_timeout_ms = DEFAULT_WATCHDOG_TIMEOUT_MS,
                             uint32_t telemetry_period_ms = DEFAULT_TELEMETRY_PERIOD_MS,
                             uint8_t invalid_command_policy = INVALID_COMMAND_POLICY);

    // Checks one incoming command (gates 1-3, see VERDICT_* in rover_protocol.h).
    // Returns true if it was accepted or was a STOP. Only those reset the timeout.
    bool ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len);

    // --- command arbiter (Task 6 A) ---
    uint8_t  lastVerdict() const { return last_verdict_; }
    bool     stopLatched() const { return stop_latched_; }
    uint32_t framesRefused() const { return frames_refused_; }
    uint8_t  invalidCommandPolicy() const { return invalid_command_policy_; }
    bool     modeConflict() const;

    void updateFaults(uint16_t hardware_faults, uint8_t controller_health);

    void updateFeedback(uint32_t encoder_update_ms, bool encoder_sane,
                        uint32_t steer_update_ms,  bool steer_sane);

    uint16_t encoderAgeMs() const;
    uint16_t steerAgeMs() const;

    bool encoderStale() const;
    bool steerStale() const;

    bool encoderInvalid() const;
    bool steerInvalid() const;

    uint16_t cmdAgeMs() const;
    bool watchdogTripped() const;

    uint8_t activeMode() const;

    bool modeRefused() const;
    uint16_t latchedFaults() const {return latched_faults_;}

    bool effectiveStop() const;

    void commandedOutputs(int16_t& drive, int16_t& steer) const;

    bool commandAuthorized() const;

    bool sourceMismatch() const;

    uint8_t commandSource() const;

    uint8_t commandStatus() const;

    bool returnRequestActive() const;

    bool telemetryDue();

    uint8_t telemetrySeq() const { return telemetry_seq_; }

    TelemetryDriveL buildDriveL(int32_t enc_left, int16_t steer_fb) const;
    TelemetryDriveR buildDriveR(int32_t enc_right) const;
    TelemetryPower  buildPower(int16_t current_ca, int16_t voltage_cv,
                               uint16_t sensor_faults = 0,
                               uint16_t extra_faults = 0) const;
    TelemetryState  buildState(uint8_t controller_health = CTRL_HEALTH_NOT_REPORTED) const;

    TelemetryCmdEcho buildCmdEcho(int16_t drive_applied, int16_t steer_applied) const;

    TelemetryFeedback buildFeedback() const;

    uint16_t faultWord(uint16_t sensor_faults = 0, uint16_t extra_faults = 0) const;

    uint8_t jetsonLinkState() const;

    uint8_t c2LinkState() const;

    uint8_t indicatorState() const;

    const ControlMsg& lastControl() const { return last_control_; }
    bool autonomyAbort() const { return last_control_.autonomy_abort; }
    bool returnRequest() const { return last_control_.return_request; }
    bool c2Lost() const { return last_control_.c2_lost; }
    bool commandFromAutonomy() const { return last_control_.from_autonomy; }

    uint32_t controlFramesAccepted() const { return control_frames_accepted_; }
    uint32_t framesIgnored() const { return frames_ignored_; }
    uint32_t seqGapsSeen() const { return seq_gaps_seen_; }
    uint32_t crcErrorsSeen() const { return crc_errors_seen_; }
    uint32_t framesLostEstimate() const { return frames_lost_estimate_; }

    DecodeResult lastDecodeResult() const { return last_decode_; }

    int16_t getCommandedDrive() const;
    int16_t getCommandedSteer() const;

    int16_t getMeasuredDrive() const { return measured_drive_; }
    int16_t getMeasuredSteer() const { return measured_steering_; }

    void updateMeasured(int16_t drive, int16_t steer);

private:
    // The only command the motors follow. Only the arbiter may change it;
    // teleop and autonomy never write it directly.
    struct ActiveCommand {
        int16_t drive;
        int16_t steer;
        uint8_t mode;
        bool    from_autonomy;
        bool    valid;
    };

    uint32_t elapsedSinceAccepted() const;
    bool recentlyDegraded() const;
    bool rxFresh() const;
    bool heldWithin(bool ever, uint32_t since_ms) const;

    uint8_t arbitrate(const ControlMsg& msg) const;
    bool modeChangeAllowed(const ControlMsg& msg) const;
    void applyMode(uint8_t to);
    void setActiveCommand(const ControlMsg& msg);
    void clearActiveCommand();
    void refuse(uint8_t verdict);
    void applyInvalidCommandPolicy();
    bool activeCommandValid() const;

    uint16_t faultCauses() const;
    bool hardwareFaultActive() const;
    bool faultConditionPresent() const;
    void latchIfFaulted();

    MillisFn now_ms_;
    uint32_t watchdog_timeout_ms_;
    uint32_t telemetry_period_ms_;
    uint8_t  invalid_command_policy_;

    uint8_t    mode_;
    uint16_t   latched_faults_;
    uint16_t   hw_faults_;
    uint8_t    hw_health_;

    ControlMsg last_control_;
    bool       have_control_;
    uint32_t   last_rx_ms_;
    bool       have_accepted_;
    uint32_t   last_accepted_ms_;

    ActiveCommand active_;
    bool       stop_latched_;
    uint8_t    last_verdict_;
    uint8_t    ack_verdict_;
    bool       ever_wrong_source_;
    uint32_t   last_wrong_source_ms_;
    bool       ever_mode_conflict_;
    uint32_t   last_mode_conflict_ms_;
    uint32_t   frames_refused_;
    uint32_t   next_telemetry_ms_;
    uint8_t    telemetry_seq_;

    bool       have_seq_;
    uint8_t    last_seq_;

    DecodeResult last_decode_;
    bool       protocol_error_;
    bool       crc_error_;
    bool       seq_gap_;
    uint32_t   last_degraded_ms_;
    bool       ever_degraded_;

    uint32_t   control_frames_accepted_;
    uint32_t   frames_ignored_;
    uint32_t   seq_gaps_seen_;
    uint32_t   crc_errors_seen_;
    uint32_t   frames_lost_estimate_;

    int16_t    measured_drive_;
    int16_t    measured_steering_;

    bool       have_encoder_fb_;
    bool       have_steer_fb_;
    uint32_t   encoder_update_ms_;
    uint32_t   steer_update_ms_;
    bool       encoder_sane_;
    bool       steer_sane_;
};
}

#endif
