#include "rover_controller.h"
#include <cstdint>

namespace rover {
RoverController::RoverController(MillisFn now_ms,
                                 uint32_t watchdog_timeout_ms,
                                 uint32_t telemetry_period_ms,
                                 uint8_t invalid_command_policy)
    : now_ms_(now_ms),
      watchdog_timeout_ms_(watchdog_timeout_ms),
      telemetry_period_ms_(telemetry_period_ms),
      invalid_command_policy_(invalid_command_policy),
      mode_(MODE_SAFE),
      latched_faults_(0),
      hw_faults_(0),
      hw_health_(CTRL_HEALTH_NOT_REPORTED),
      have_control_(false),
      last_rx_ms_(0),
      have_accepted_(false),
      last_accepted_ms_(0),
      stop_latched_(true),
      last_verdict_(VERDICT_NONE),
      ack_verdict_(VERDICT_NONE),
      ever_wrong_source_(false),
      last_wrong_source_ms_(0),
      ever_mode_conflict_(false),
      last_mode_conflict_ms_(0),
      frames_refused_(0),
      next_telemetry_ms_(now_ms ? now_ms() : 0),
      telemetry_seq_(0),
      have_seq_(false),
      last_seq_(0),
      last_decode_(DECODE_OK),
      protocol_error_(false),
      crc_error_(false),
      seq_gap_(false),
      last_degraded_ms_(0),
      ever_degraded_(false),
      control_frames_accepted_(0),
      frames_ignored_(0),
      seq_gaps_seen_(0),
      crc_errors_seen_(0),
      frames_lost_estimate_(0),
      measured_drive_(0),
      measured_steering_(0),
      have_encoder_fb_(false),
      have_steer_fb_(false),
      encoder_update_ms_(0),
      steer_update_ms_(0),
      encoder_sane_(false),
      steer_sane_(false) {
    last_control_.drive_cmd         = 0;
    last_control_.steer_cmd         = 0;
    last_control_.mode              = MODE_SAFE;
    last_control_.stop              = true;
    last_control_.autonomy_abort    = false;
    last_control_.return_request    = false;
    last_control_.indicator_request = INDICATOR_OFF;
    last_control_.c2_lost           = true;
    last_control_.from_autonomy     = false;
    last_control_.seq               = 0;
    clearActiveCommand();
}

bool RoverController::ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len) {
    if (can_id != CAN_ID_CONTROL) { frames_ignored_++; return false; }

    ControlMsg msg;
    const DecodeResult r = decodeControl(can_id, buf, len, msg);

    latchIfFaulted();
    last_decode_ = r;

    // Gates 1 and 2: did it arrive undamaged, and are all its values allowed?
    if (r != DECODE_OK) {
        frames_ignored_++;
        if (r == DECODE_BAD_CRC) {
            crc_error_ = true;
            crc_errors_seen_++;
            last_degraded_ms_ = now_ms_();
            ever_degraded_ = true;
        } else {
            protocol_error_ = true;
        }
        const bool corrupt = (r == DECODE_BAD_CRC || r == DECODE_BAD_DLC);
        last_verdict_ = corrupt ? VERDICT_REJECT_CORRUPT : VERDICT_REJECT_MALFORMED;
        if (!corrupt) applyInvalidCommandPolicy();
        return false;
    }

    if (have_seq_) {
        const uint8_t delta = seqDelta(last_seq_, msg.seq);
        if (delta != 1) {
            seq_gap_ = true;
            seq_gaps_seen_++;
            if (delta > 1) frames_lost_estimate_ += static_cast<uint32_t>(delta - 1);
            last_degraded_ms_ = now_ms_();
            ever_degraded_ = true;
        } else {
            seq_gap_ = false;
        }
    }
    last_seq_ = msg.seq;
    have_seq_ = true;

    protocol_error_ = false;
    crc_error_      = false;
    last_control_   = msg;
    have_control_   = true;
    last_rx_ms_     = now_ms_();

    // Gate 3: is this sender allowed to ask for this?
    const uint8_t verdict = arbitrate(msg);
    last_verdict_ = verdict;
    ack_verdict_  = verdict;
    if (verdictIsRejection(verdict)) {
        refuse(verdict);
        latchIfFaulted();
        return false;
    }

    // STOP never changes the mode. A mode change in the same frame must
    // still be allowed on its own.
    if (modeChangeAllowed(msg)) applyMode(msg.mode);

    if (verdict == VERDICT_STOP) {
        stop_latched_ = true;
        clearActiveCommand();
    } else {
        setActiveCommand(msg);
        // Only a neutral command (drive 0, steer 0) turns STOP off,
        // so the rover can never restart by surprise.
        if (msg.drive_cmd == 0 && msg.steer_cmd == 0) stop_latched_ = false;
    }

    have_accepted_    = true;
    last_accepted_ms_ = now_ms_();
    control_frames_accepted_++;
    latchIfFaulted();
    return true;
}

uint8_t RoverController::arbitrate(const ControlMsg& msg) const {
    // STOP beats every mode and every sender.
    // In AUTONOMOUS, an abort counts as a STOP.
    if (msg.stop) return VERDICT_STOP;
    if (msg.autonomy_abort && mode_ == MODE_AUTONOMOUS) return VERDICT_STOP;

    if (!modeChangeAllowed(msg)) return VERDICT_REJECT_MODE_REFUSED;

    const bool motion = msg.drive_cmd != 0 || msg.steer_cmd != 0;
    if (!modePermitsMotion(msg.mode)) {
        return motion ? VERDICT_REJECT_MODE_CONFLICT : VERDICT_ACCEPTED;
    }
    if (sourceOwnsMode(msg.from_autonomy, msg.mode)) return VERDICT_ACCEPTED;

    // A sender may switch the rover into a mode it doesn't own (the operator
    // handing over to AUTONOMOUS), but may not drive in that mode.
    if (msg.mode != mode_ && !motion) return VERDICT_ACCEPTED;
    return VERDICT_REJECT_WRONG_SOURCE;
}

void RoverController::setActiveCommand(const ControlMsg& msg) {
    if (!sourceOwnsMode(msg.from_autonomy, mode_)) {
        clearActiveCommand();
        return;
    }
    active_.drive         = msg.drive_cmd;
    active_.steer         = msg.steer_cmd;
    active_.mode          = mode_;
    active_.from_autonomy = msg.from_autonomy;
    active_.valid         = true;
}

void RoverController::clearActiveCommand() {
    active_.drive         = 0;
    active_.steer         = 0;
    active_.mode          = MODE_SAFE;
    active_.from_autonomy = false;
    active_.valid         = false;
}

void RoverController::refuse(uint8_t verdict) {
    frames_refused_++;
    const uint32_t now = now_ms_();
    if (verdict == VERDICT_REJECT_WRONG_SOURCE) {
        ever_wrong_source_    = true;
        last_wrong_source_ms_ = now;
    } else {
        ever_mode_conflict_    = true;
        last_mode_conflict_ms_ = now;
    }
    applyInvalidCommandPolicy();
}

void RoverController::applyInvalidCommandPolicy() {
    if (invalid_command_policy_ != INVALID_CMD_STOP) return;
    stop_latched_ = true;
    clearActiveCommand();
}

bool RoverController::activeCommandValid() const {
    if (!active_.valid) return false;
    if (active_.mode != activeMode()) return false;
    return sourceOwnsMode(active_.from_autonomy, active_.mode);
}

bool RoverController::heldWithin(bool ever, uint32_t since_ms) const {
    if (!ever) return false;
    return (now_ms_() - since_ms) < REJECT_REPORT_HOLD_MS;
}

bool RoverController::rxFresh() const {
    if (!have_control_) return false;
    return (now_ms_() - last_rx_ms_) < watchdog_timeout_ms_;
}

void RoverController:: updateFaults(uint16_t hardware_faults, uint8_t controller_health) {
    hw_faults_ = hardware_faults;
    hw_health_ = controller_health;
    latchIfFaulted();
}

bool RoverController::modeChangeAllowed(const ControlMsg& msg) const {
    const uint8_t to = msg.mode;
    if (to == mode_) return true;
    if (!transitionAllowed(mode_, to, msg.from_autonomy)) return false;
    if (mode_ == MODE_FAULT && hardwareFaultActive()) return false;
    if (to == MODE_TELEOP && msg.c2_lost) return false;
    return true;
}

void RoverController::applyMode(uint8_t to) {
    if (to == mode_) return;
    mode_ = to;
    if (to == MODE_SAFE) {
        // Going to SAFE is the reset button: clear all stored faults.
        latched_faults_     = 0;
        ever_wrong_source_  = false;
        ever_mode_conflict_ = false;
    }
}

uint16_t RoverController::faultCauses()const{
    uint16_t causes = static_cast<uint16_t>(hw_faults_ & FAULT_CRITICAL_MASK);
    if(watchdogTripped()) causes |= FAULT_COMM_TIMEOUT;
    if(mode_ == MODE_TELEOP && last_control_.c2_lost) causes |= FAULT_C2_LINK_LOST;
    return causes;
}

bool RoverController::hardwareFaultActive()const{
    return (hw_faults_ & FAULT_CRITICAL_MASK) != 0 || hw_health_ == CTRL_HEALTH_FAULT;
}

bool RoverController::faultConditionPresent()const{
    if(!modePermitsMotion(mode_)) return false;
    return faultCauses() != 0 || hw_health_ == CTRL_HEALTH_FAULT;
}

void RoverController::latchIfFaulted(){
    if(!faultConditionPresent()) return;
    latched_faults_ = faultCauses();
    mode_ = MODE_FAULT;
}

uint32_t RoverController::elapsedSinceAccepted() const {
    return now_ms_() - last_accepted_ms_;
}

uint16_t RoverController::cmdAgeMs() const {
    if (!have_accepted_) return CMD_AGE_UNKNOWN;
    return clampCmdAgeMs(static_cast<int64_t>(elapsedSinceAccepted()));
}

bool RoverController::watchdogTripped() const {
    if (!have_accepted_) return true;
    return elapsedSinceAccepted() >= watchdog_timeout_ms_;
}

uint8_t RoverController::activeMode() const {
    if(faultConditionPresent()) return MODE_FAULT;
    return mode_;
}

bool RoverController::modeRefused() const {
    return rxFresh() && ack_verdict_ == VERDICT_REJECT_MODE_REFUSED;
}

bool RoverController::modeConflict() const {
    return heldWithin(ever_mode_conflict_, last_mode_conflict_ms_);
}

// Gate 4, checked every loop: is the rover allowed to move at all right now?
bool RoverController::effectiveStop() const {
    return stop_latched_ || !modePermitsMotion(activeMode());
}

bool RoverController::commandAuthorized() const {
    if (!have_control_) return false;
    if(last_control_.mode != activeMode()) return false;
    return sourceOwnsMode(last_control_.from_autonomy, activeMode());
}

bool RoverController::sourceMismatch() const {
    return heldWithin(ever_wrong_source_, last_wrong_source_ms_);
}

uint8_t RoverController::commandStatus() const {
    if (!have_control_) return CMD_STATUS_NO_COMMAND;
    if (rxFresh()) {
        switch (ack_verdict_) {
            case VERDICT_REJECT_MODE_REFUSED:  return CMD_STATUS_MODE_REFUSED;
            case VERDICT_REJECT_MODE_CONFLICT: return CMD_STATUS_MODE_CONFLICT;
            case VERDICT_REJECT_WRONG_SOURCE:  return CMD_STATUS_WRONG_SOURCE;
        }
    }
    if (effectiveStop())       return CMD_STATUS_STOPPED;
    if (!activeCommandValid()) return CMD_STATUS_NO_COMMAND;
    return CMD_STATUS_APPLIED;
}

bool RoverController::returnRequestActive() const {
    if (!have_control_ || watchdogTripped()) return false;
    return last_control_.return_request && activeMode() == MODE_AUTONOMOUS;
}

void RoverController::commandedOutputs(int16_t& drive, int16_t& steer) const {
    if (effectiveStop() || !activeCommandValid()) { drive = 0; steer = 0; return; }
    drive = active_.drive;
    steer = active_.steer;
}

bool RoverController::telemetryDue() {
    const uint32_t now = now_ms_();
    const int32_t since_due = static_cast<int32_t>(now - next_telemetry_ms_);
    if (since_due < 0) return false;

    next_telemetry_ms_ += telemetry_period_ms_;
    if (static_cast<int32_t>(now - next_telemetry_ms_)
            > static_cast<int32_t>(telemetry_period_ms_)) {
        next_telemetry_ms_ = now + telemetry_period_ms_;
    }
    telemetry_seq_++;
    return true;
}

bool RoverController::recentlyDegraded() const {
    if (!ever_degraded_) return false;
    return (now_ms_() - last_degraded_ms_) < LINK_DEGRADED_HOLD_MS;
}

// Is the Jetson still talking? Separate from the command timeout: a Jetson
// sending only rejected commands is alive, but has no valid command.
uint8_t RoverController::jetsonLinkState() const {
    if (!rxFresh()) return LINK_LOST;
    if (recentlyDegraded()) return LINK_DEGRADED;
    return LINK_OK;
}

uint8_t RoverController::c2LinkState() const {
    if (!have_control_)    return LINK_NOT_REPORTED;
    if (watchdogTripped()) return LINK_NOT_REPORTED;
    return last_control_.c2_lost ? LINK_LOST : LINK_OK;
}

uint16_t RoverController::faultWord(uint16_t sensor_faults, uint16_t extra_faults) const {
    uint16_t faults = static_cast<uint16_t>(sensor_faults | extra_faults);
    if (watchdogTripped()) faults |= FAULT_COMM_TIMEOUT;
    faults |= latched_faults_;
    if (encoderStale() || steerStale()) faults |= FAULT_STALE_FEEDBACK;
    if (encoderInvalid())               faults |= FAULT_ENCODER_FAULT;
    if (steerInvalid())                 faults |= FAULT_STEERING_FAULT;
    if (protocol_error_)   faults |= FAULT_PROTOCOL_ERROR;
    if (crc_error_)        faults |= FAULT_CRC_ERROR;
    if (seq_gap_)          faults |= FAULT_SEQ_GAP;
    if (c2LinkState() == LINK_LOST) faults |= FAULT_C2_LINK_LOST;
    if (sourceMismatch()) faults |= FAULT_CMD_SOURCE;
    if (modeConflict())   faults |= FAULT_MODE_CONFLICT;
    return faults;
}

uint8_t RoverController::indicatorState() const {
    const uint8_t mode = activeMode();
    if (mode == MODE_FAULT) return INDICATOR_FAULT;

    if (mode == MODE_SAFE) return INDICATOR_OFF;

    if (mode == MODE_TELEOP) return INDICATOR_TELEOP;

    if (commandAuthorized()
            && last_control_.indicator_request == INDICATOR_ARRIVED) {
        return INDICATOR_ARRIVED;
    }
    return INDICATOR_AUTONOMOUS;
}

TelemetryDriveL RoverController::buildDriveL(int32_t enc_left, int16_t steer_fb) const {
    TelemetryDriveL m;
    m.enc_left = enc_left;
    m.steer_fb = steer_fb;
    return m;
}

TelemetryDriveR RoverController::buildDriveR(int32_t enc_right) const {
    TelemetryDriveR m;
    m.enc_right  = enc_right;
    m.cmd_age_ms = cmdAgeMs();
    return m;
}

TelemetryPower RoverController::buildPower(int16_t current_ca, int16_t voltage_cv,
                                           uint16_t sensor_faults,
                                           uint16_t extra_faults) const {
    TelemetryPower m;
    m.current_ca   = current_ca;
    m.voltage_cv   = voltage_cv;
    m.fault_status = faultWord(sensor_faults, extra_faults);
    return m;
}

TelemetryState RoverController::buildState(uint8_t controller_health) const {
    TelemetryState m;
    m.mode              = activeMode();
    m.jetson_link       = jetsonLinkState();
    m.c2_link           = c2LinkState();
    m.controller_health = controller_health;
    m.indicator_state   = indicatorState();
    m.reserved          = 0;
    return m;
}

TelemetryCmdEcho RoverController::buildCmdEcho(int16_t drive_applied,
                                               int16_t steer_applied) const {
    TelemetryCmdEcho m;
    m.drive_applied = drive_applied;
    m.steer_applied = steer_applied;
    m.ack_seq = last_control_.seq;
    m.status  = packCmdStatus(commandStatus(), last_decode_, returnRequestActive());
    return m;
}

int16_t RoverController::getCommandedDrive() const {
    int16_t drive, steer;
    commandedOutputs(drive, steer);
    return drive;
}

int16_t RoverController::getCommandedSteer() const {
    int16_t drive, steer;
    commandedOutputs(drive, steer);
    return steer;
}

void RoverController::updateMeasured(int16_t drive, int16_t steer) {
    measured_drive_    = drive;
    measured_steering_ = steer;
}

void RoverController::updateFeedback(uint32_t encoder_update_ms, bool encoder_sane,
                                     uint32_t steer_update_ms,  bool steer_sane) {
    encoder_update_ms_ = encoder_update_ms;
    steer_update_ms_   = steer_update_ms;
    encoder_sane_      = encoder_sane;
    steer_sane_        = steer_sane;
    have_encoder_fb_   = true;
    have_steer_fb_     = true;
}

uint16_t RoverController::encoderAgeMs() const {
    if (!have_encoder_fb_) return FEEDBACK_AGE_UNKNOWN;
    return clampFeedbackAgeMs(
        static_cast<int64_t>(now_ms_() - encoder_update_ms_));
}

uint16_t RoverController::steerAgeMs() const {
    if (!have_steer_fb_) return FEEDBACK_AGE_UNKNOWN;
    return clampFeedbackAgeMs(
        static_cast<int64_t>(now_ms_() - steer_update_ms_));
}

bool RoverController::encoderStale() const {
    if (!have_encoder_fb_) return true;
    return (now_ms_() - encoder_update_ms_) >= FEEDBACK_STALE_MS;
}

bool RoverController::steerStale() const {
    if (!have_steer_fb_) return true;
    return (now_ms_() - steer_update_ms_) >= FEEDBACK_STALE_MS;
}

bool RoverController::encoderInvalid() const {
    return have_encoder_fb_ && !encoder_sane_;
}

bool RoverController::steerInvalid() const {
    return have_steer_fb_ && !steer_sane_;
}

uint8_t RoverController::commandSource() const {
    if (watchdogTripped() || effectiveStop()) return CMD_SOURCE_NONE;
    if (!activeCommandValid())                return CMD_SOURCE_NONE;
    return active_.from_autonomy ? CMD_SOURCE_AUTONOMY : CMD_SOURCE_OPERATOR;
}

TelemetryFeedback RoverController::buildFeedback() const {
    TelemetryFeedback m;
    m.encoder_age_ms = encoderAgeMs();
    m.steer_age_ms   = steerAgeMs();
    m.cmd_source     = commandSource();
    uint8_t flags = 0;
    if (encoderStale())   flags |= FB_FLAG_ENCODER_STALE;
    if (steerStale())     flags |= FB_FLAG_STEER_STALE;
    if (encoderInvalid()) flags |= FB_FLAG_ENCODER_INVALID;
    if (steerInvalid())   flags |= FB_FLAG_STEER_INVALID;
    m.flags = flags;
    return m;
}
}
