#include "rover_controller.h"
#include <cstdint>

namespace rover {
RoverController::RoverController(MillisFn now_ms,
                                 uint32_t watchdog_timeout_ms,
                                 uint32_t telemetry_period_ms)
    : now_ms_(now_ms),
      watchdog_timeout_ms_(watchdog_timeout_ms),
      telemetry_period_ms_(telemetry_period_ms),
      mode_(MODE_SAFE),
      latched_faults_(0),
      hw_faults_(0),
      hw_health_(CTRL_HEALTH_NOT_REPORTED),
      have_control_(false),
      last_control_ms_(0),
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
}

bool RoverController::ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len) {
    if (can_id != CAN_ID_CONTROL) { frames_ignored_++; return false; }

    ControlMsg msg;
    const DecodeResult r = decodeControl(can_id, buf, len, msg);

    latchIfFaulted();
    last_decode_ = r;

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

    protocol_error_  = false;
    crc_error_       = false;
    last_control_    = msg;
    have_control_    = true;
    last_control_ms_ = now_ms_();
    control_frames_accepted_++;
    requestMode(msg);
    latchIfFaulted();
    return true;
}

void RoverController:: updateFaults(uint16_t hardware_faults, uint8_t controller_health) {
    hw_faults_ = hardware_faults;
    hw_health_ = controller_health;
    latchIfFaulted();
}

void RoverController::requestMode(const ControlMsg& msg){
    const uint8_t to = msg.mode;
    if(to == mode_) return;
    if(!transitionAllowed(mode_, to, msg.from_autonomy)) return;
    if(mode_ == MODE_FAULT && hardwareFaultActive()) return;
    if(to ==MODE_TELEOP && msg.c2_lost) return;
    mode_ = to;
    if(to == MODE_SAFE) latched_faults_ = 0;
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

uint32_t RoverController::elapsedSinceControl() const {
    return now_ms_() - last_control_ms_;
}

uint16_t RoverController::cmdAgeMs() const {
    if (!have_control_) return CMD_AGE_UNKNOWN;
    return clampCmdAgeMs(static_cast<int64_t>(elapsedSinceControl()));
}

bool RoverController::watchdogTripped() const {
    if (!have_control_) return true;
    return elapsedSinceControl() >= watchdog_timeout_ms_;
}

uint8_t RoverController::activeMode() const {
    if(faultConditionPresent()) return MODE_FAULT;
    return mode_;
}

bool RoverController::modeRefused() const{
    if(!have_control_ || watchdogTripped()) return false;
    return last_control_.mode != activeMode();
}

bool RoverController::effectiveStop() const {
    return last_control_.stop
        || !modePermitsMotion(activeMode())
        || (last_control_.autonomy_abort && activeMode() == MODE_AUTONOMOUS);
}

bool RoverController::commandAuthorized() const {
    if (!have_control_) return false;
    if(last_control_.mode != activeMode()) return false;
    return sourceOwnsMode(last_control_.from_autonomy, activeMode());
}

bool RoverController::sourceMismatch() const {
    if (!modePermitsMotion(activeMode())) return false;
    if(modeRefused() || last_control_.stop) return false;
    return !commandAuthorized();
}

uint8_t RoverController::commandStatus() const {
    if (!have_control_)       return CMD_STATUS_NO_COMMAND;
    if(modeRefused())         return CMD_STATUS_MODE_REFUSED;
    if (effectiveStop())      return CMD_STATUS_STOPPED;
    if (!commandAuthorized()) return CMD_STATUS_WRONG_SOURCE;
    return CMD_STATUS_APPLIED;
}

bool RoverController::returnRequestActive() const {
    if (!have_control_ || watchdogTripped()) return false;
    return last_control_.return_request && activeMode() == MODE_AUTONOMOUS;
}

void RoverController::commandedOutputs(int16_t& drive, int16_t& steer) const {
    if (effectiveStop() || !commandAuthorized()) { drive = 0; steer = 0; return; }
    drive = last_control_.drive_cmd;
    steer = last_control_.steer_cmd;
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

uint8_t RoverController::jetsonLinkState() const {
    if (watchdogTripped()) return LINK_LOST;
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
    if (!have_control_ || watchdogTripped())      return CMD_SOURCE_NONE;
    if (!modePermitsMotion(activeMode()))          return CMD_SOURCE_NONE;
    if (!commandAuthorized())                      return CMD_SOURCE_NONE;
    return last_control_.from_autonomy ? CMD_SOURCE_AUTONOMY : CMD_SOURCE_OPERATOR;
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
