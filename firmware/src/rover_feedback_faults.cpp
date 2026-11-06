#include "rover_feedback_faults.h"

namespace rover {

namespace {

struct FaultName { uint8_t fault; const char* name; };

const FaultName FAULT_NAMES[] = {
    {FB_FAULT_NONE,               "none"},
    {FB_FAULT_ENCODER_FROZEN,     "encoder-frozen"},
    {FB_FAULT_STEER_FROZEN,       "steer-frozen"},
    {FB_FAULT_ENCODER_IMPOSSIBLE, "encoder-impossible"},
    {FB_FAULT_STEER_IMPOSSIBLE,   "steer-impossible"},
    {FB_FAULT_DELAYED,            "delayed"},
};

bool sameName(const char* a, const char* b) {
    while (*a && *b) { if (*a++ != *b++) return false; }
    return *a == *b;
}

}  // namespace

const char* feedbackFaultName(uint8_t fault) {
    for (const FaultName& f : FAULT_NAMES) {
        if (f.fault == fault) return f.name;
    }
    return "unknown";
}

bool parseFeedbackFault(const char* name, uint8_t& out) {
    if (name == nullptr) return false;
    for (const FaultName& f : FAULT_NAMES) {
        if (sameName(name, f.name)) { out = f.fault; return true; }
    }
    return false;
}

FeedbackFaultInjector::FeedbackFaultInjector()
    : fault_(FB_FAULT_NONE),
      have_frozen_(false),
      count_(0),
      next_(0) {
    frozen_encoder_.count = 0;
    frozen_encoder_.rpm = 0;
    frozen_encoder_.last_update_ms = 0;
    frozen_encoder_.valid = false;
    frozen_steer_.commanded = 0;
    frozen_steer_.measured = 0;
    frozen_steer_.error = 0;
    frozen_steer_.last_update_ms = 0;
    frozen_steer_.valid = false;
}

void FeedbackFaultInjector::arm(uint8_t fault) {
    if (fault == fault_) return;
    fault_ = (fault > FB_FAULT_MAX) ? static_cast<uint8_t>(FB_FAULT_NONE) : fault;
    // The frozen snapshot belongs to one arming. Dropping it here is what
    // makes clearing a fault and re-arming it freeze the CURRENT reading
    // rather than resurrect one from minutes ago.
    have_frozen_ = false;
}

void FeedbackFaultInjector::record(const EncoderFeedback& e,
                                   const SteerFeedback& s, uint32_t now_ms) {
    if (count_ > 0) {
        const uint8_t last =
            static_cast<uint8_t>((next_ + FB_FAULT_HISTORY - 1) % FB_FAULT_HISTORY);
        // Elapsed-time subtraction, so this survives the millis() wrap.
        if ((now_ms - history_[last].taken_ms) < FB_FAULT_SAMPLE_MS) return;
    }
    history_[next_].taken_ms = now_ms;
    history_[next_].encoder  = e;
    history_[next_].steer    = s;
    next_ = static_cast<uint8_t>((next_ + 1) % FB_FAULT_HISTORY);
    if (count_ < FB_FAULT_HISTORY) count_++;
}

const FeedbackFaultInjector::Sample*
FeedbackFaultInjector::due(uint32_t now_ms) const {
    // Newest sample at least FB_FAULT_DELAY_MS old. Walking backwards from
    // the most recent means the first match is the newest one, so the delayed
    // channel runs exactly FB_FAULT_DELAY_MS behind rather than further.
    const Sample* oldest = nullptr;
    for (uint8_t i = 0; i < count_; ++i) {
        const uint8_t idx =
            static_cast<uint8_t>((next_ + FB_FAULT_HISTORY - 1 - i) % FB_FAULT_HISTORY);
        const Sample& s = history_[idx];
        // Elapsed-time subtraction, never a timestamp comparison, so this
        // survives the millis() wrap like everything else here.
        if ((now_ms - s.taken_ms) >= FB_FAULT_DELAY_MS) return &s;
        oldest = &s;
    }
    // Nothing that old yet: the oldest we have is the most delayed answer
    // available, which is what a real delay line does while it fills.
    return oldest;
}

void FeedbackFaultInjector::apply(EncoderFeedback& encoder, SteerFeedback& steer,
                                  uint32_t now_ms) {
    record(encoder, steer, now_ms);

    switch (fault_) {
        case FB_FAULT_NONE:
            return;

        case FB_FAULT_ENCODER_FROZEN:
            if (!have_frozen_) {
                frozen_encoder_ = encoder;
                have_frozen_ = true;
            }
            // The timestamp freezes with the value. That is the whole point:
            // the reading stays perfectly plausible and only its age shows
            // that the channel died.
            encoder = frozen_encoder_;
            return;

        case FB_FAULT_STEER_FROZEN:
            if (!have_frozen_) {
                frozen_steer_ = steer;
                have_frozen_ = true;
            }
            steer = frozen_steer_;
            return;

        case FB_FAULT_ENCODER_IMPOSSIBLE:
            // An order of magnitude past full scale -- not a plausible
            // overspeed, but a number the wheel could not produce at all,
            // which is what a disconnected sensor or a lost decode looks
            // like. The timestamp stays current: this value is fresh and
            // wrong, which is why invalid is a separate flag from stale.
            encoder.rpm = static_cast<int16_t>(ENCODER_MAX_RPM * 10);
            return;

        case FB_FAULT_STEER_IMPOSSIBLE:
            steer.measured = static_cast<int16_t>(CMD_MAX * 2);
            steer.error = static_cast<int16_t>(steer.commanded - steer.measured);
            return;

        case FB_FAULT_DELAYED: {
            const Sample* s = due(now_ms);
            if (s != nullptr) {
                encoder = s->encoder;
                steer   = s->steer;
            }
            return;
        }

        default:
            return;
    }
}

}  // namespace rover
