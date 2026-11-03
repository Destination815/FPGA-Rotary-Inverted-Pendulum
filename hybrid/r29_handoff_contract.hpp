#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>

// Q18 measured states; already capped/slew-limited integer permille commands.
// No observer gains, microcode, or plant parameters are modified here.
struct RescueHandoff {
    bool latched=false, eligible=false, activated=false, released=false;
    int opposed_ticks=0, stable_ticks=0, compatible_ticks=0, fast_compatible_ticks=0, rescue_ticks=0;
    void reset() {
        latched=eligible=activated=released=false;
        opposed_ticks=stable_ticks=compatible_ticks=fast_compatible_ticks=rescue_ticks=0;
    }
    int tick(int mode, int64_t theta, int64_t omega, int64_t velocity,
             bool observing, bool measurement_valid, int mixed, int fallback,
             bool stop_or_reset=false, bool early_trigger=false, bool fast_release_permission=false) {
        activated=released=false;
        if(stop_or_reset || mode!=2 || !observing) { reset(); return mixed; }
        // Equivalent to the original fixed early_guard_root pre-tick force.
        // Do not mark activated: the original tick had just cleared it.
        if(early_trigger) {
            latched=true; opposed_ticks=stable_ticks=compatible_ticks=fast_compatible_ticks=rescue_ticks=0;
        }
        if(!measurement_valid) {
            eligible=false; opposed_ticks=stable_ticks=compatible_ticks=fast_compatible_ticks=rescue_ticks=0;
            return latched?fallback:mixed;
        }
        const bool departing=(theta>0 && omega>0)||(theta<0 && omega<0);
        const bool opposite=(theta>0 && mixed<0)||(theta<0 && mixed>0);
        eligible=std::abs(theta)>13726 && departing && opposite;
        if(!latched) {
            opposed_ticks=eligible?std::min(20,opposed_ticks+1):0;
            if(opposed_ticks>=20) { latched=true; stable_ticks=compatible_ticks=fast_compatible_ticks=rescue_ticks=0; activated=true; }
        }
        if(latched) {
            const bool stable=std::abs(theta)<6863 && std::abs(omega)<52429
                              && std::abs(velocity)<78643;
            stable_ticks=stable?std::min(100,stable_ticks+1):0;
            const bool inward=(theta>=0 && omega<=0)||(theta<=0 && omega>=0);
            const bool same=(mixed>0 && fallback>0)||(mixed<0 && fallback<0);
            const bool compatible=rescue_ticks>=500 && std::abs(theta)<11438 && std::abs(omega)<262144 &&
                std::abs(velocity)<393216 && inward && same && std::abs(mixed-fallback)<=20;
            const bool fast_compatible=fast_release_permission && rescue_ticks>=100 && std::llabs(theta)<39322 && std::llabs(omega)<655360 && std::llabs(velocity)<786432 && inward && same && std::llabs(int64_t(mixed)-fallback)<=125;
            fast_compatible_ticks=fast_compatible?std::min(20,fast_compatible_ticks+1):0;
            if(!activated)rescue_ticks=std::min(500,rescue_ticks+1);
            compatible_ticks=compatible?std::min(20,compatible_ticks+1):0;
            if(stable_ticks>=100 || compatible_ticks>=20 || fast_compatible_ticks>=20) {
                latched=false; opposed_ticks=stable_ticks=compatible_ticks=fast_compatible_ticks=rescue_ticks=0; released=true;
            }
            // Release frame is still fallback. Caller resets original blend
            // weight to zero; its existing +83886/Q24 ramp resumes next frame.
            return fallback;
        }
        stable_ticks=compatible_ticks=fast_compatible_ticks=rescue_ticks=0;
        return mixed;
    }
};

// Native frame-level counterpart; previous_cap belongs to previous_command.
// Controller RTL additionally holds a pending force until guard completion.
struct EarlySaturation {
    int count=0;
    bool fired=false, prior_mature=false;
    void reset(){count=0;fired=false;prior_mature=false;}
    bool tick(bool mature,int32_t previous_command,int32_t previous_cap) {
        if(!mature){reset();return false;}
        bool trigger=false;
        if(!fired) {
            const int64_t magnitude=std::llabs(int64_t(previous_command));
            count=prior_mature&&previous_cap>0&&magnitude>=previous_cap
                ?std::min(100,count+1):0;
            if(count>=100){fired=true;trigger=true;}
        }
        prior_mature=mature;
        return trigger;
    }
};

inline bool early_returning_gate(int32_t theta_q18,int32_t omega_q18){
    const int64_t theta_abs=std::llabs(int64_t(theta_q18));
    const int64_t omega_abs=std::llabs(int64_t(omega_q18));
    const bool strict_inward=(theta_q18>0&&omega_q18<0)||
                             (theta_q18<0&&omega_q18>0);
    return theta_abs>0&&(!strict_inward||2*theta_abs>omega_abs);
}
