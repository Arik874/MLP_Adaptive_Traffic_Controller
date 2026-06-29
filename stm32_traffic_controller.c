#include <stdint.h>
#include "stm32_traffic_controller.h"

/*
 * Full conditional traffic controller for STM32 deployment.
 *
 * This module intentionally uses deterministic if/else logic as the primary
 * scheduler and keeps green time within [5s, 30s] as described in the project
 * document. The phase order is four-way round-robin with demand-aware selection.
 */

static float clamp_unit(float value) {
    if (value < 0.0f) {
        return 0.0f;
    } else if (value > 1.0f) {
        return 1.0f;
    } else {
        return value;
    }
}

static float clamp_green_seconds(float value) {
    if (value < TRAFFIC_MIN_GREEN_SECONDS) {
        return TRAFFIC_MIN_GREEN_SECONDS;
    } else if (value > TRAFFIC_MAX_GREEN_SECONDS) {
        return TRAFFIC_MAX_GREEN_SECONDS;
    } else {
        return value;
    }
}

static float lane_density_for_phase(traffic_phase_t phase,
                                    float lane1,
                                    float lane2,
                                    float lane3,
                                    float lane4) {
    if (phase == TRAFFIC_PHASE_LANE1) {
        return lane1;
    } else if (phase == TRAFFIC_PHASE_LANE2) {
        return lane2;
    } else if (phase == TRAFFIC_PHASE_LANE3) {
        return lane3;
    } else {
        return lane4;
    }
}

static traffic_phase_t next_phase_round_robin(traffic_phase_t phase) {
    if (phase == TRAFFIC_PHASE_LANE1) {
        return TRAFFIC_PHASE_LANE2;
    } else if (phase == TRAFFIC_PHASE_LANE2) {
        return TRAFFIC_PHASE_LANE3;
    } else if (phase == TRAFFIC_PHASE_LANE3) {
        return TRAFFIC_PHASE_LANE4;
    } else {
        return TRAFFIC_PHASE_LANE1;
    }
}

static traffic_phase_t select_highest_demand_phase(float lane1,
                                                    float lane2,
                                                    float lane3,
                                                    float lane4,
                                                    traffic_phase_t tie_break_start) {
    traffic_phase_t probe = tie_break_start;
    traffic_phase_t best_phase = tie_break_start;
    float best_density = lane_density_for_phase(probe, lane1, lane2, lane3, lane4);

    for (int i = 0; i < 4; ++i) {
        float density = lane_density_for_phase(probe, lane1, lane2, lane3, lane4);
        if (density > best_density) {
            best_density = density;
            best_phase = probe;
        }
        probe = next_phase_round_robin(probe);
    }

    return best_phase;
}

static float conditional_green_time_seconds(float lane_density,
                                            float avg_density,
                                            float phase_elapsed_seconds,
                                            float default_green_seconds) {
    float green_seconds;

    if (lane_density >= 0.90f) {
        green_seconds = 30.0f;
    } else if (lane_density >= 0.75f) {
        green_seconds = 24.0f;
    } else if (lane_density >= 0.60f) {
        green_seconds = 18.0f;
    } else if (lane_density >= 0.40f) {
        green_seconds = 12.0f;
    } else if (lane_density >= 0.20f) {
        green_seconds = 8.0f;
    } else {
        green_seconds = 5.0f;
    }

    /* If intersection-wide demand is low, reduce per-phase time. */
    if (avg_density < 0.15f && green_seconds > default_green_seconds) {
        green_seconds = default_green_seconds;
    }

    /* If this phase is already long, avoid extending it further. */
    if (phase_elapsed_seconds > (green_seconds - TRAFFIC_YELLOW_SECONDS) && green_seconds > 8.0f) {
        green_seconds -= 2.0f;
    }

    return clamp_green_seconds(green_seconds);
}

void traffic_controller_init(traffic_controller_state_t *state) {
    if (state == 0) {
        return;
    }

    state->current_phase = TRAFFIC_PHASE_LANE1;
    state->current_stage = TRAFFIC_STAGE_GREEN;
    state->phase_elapsed_seconds = 0.0f;
    state->current_green_seconds = TRAFFIC_DEFAULT_GREEN_SECONDS;
}

void traffic_controller_update(traffic_controller_state_t *state,
                               float lane1_density,
                               float lane2_density,
                               float lane3_density,
                               float lane4_density,
                               float delta_seconds) {
    float lane1;
    float lane2;
    float lane3;
    float lane4;
    float avg_density;
    float active_lane_density;
    traffic_phase_t phase_after_rr;

    if (state == 0) {
        return;
    }

    lane1 = clamp_unit(lane1_density);
    lane2 = clamp_unit(lane2_density);
    lane3 = clamp_unit(lane3_density);
    lane4 = clamp_unit(lane4_density);

    if (delta_seconds > 0.0f) {
        state->phase_elapsed_seconds += delta_seconds;
    }

    avg_density = (lane1 + lane2 + lane3 + lane4) * 0.25f;
    active_lane_density = lane_density_for_phase(
        state->current_phase,
        lane1,
        lane2,
        lane3,
        lane4);

    state->current_green_seconds = conditional_green_time_seconds(
        active_lane_density,
        avg_density,
        state->phase_elapsed_seconds,
        TRAFFIC_DEFAULT_GREEN_SECONDS);

    if (state->phase_elapsed_seconds >= state->current_green_seconds) {
        state->current_stage = TRAFFIC_STAGE_YELLOW;
    } else if (state->phase_elapsed_seconds >= (state->current_green_seconds - TRAFFIC_YELLOW_SECONDS)) {
        state->current_stage = TRAFFIC_STAGE_YELLOW;
    } else {
        state->current_stage = TRAFFIC_STAGE_GREEN;
    }

    if (state->phase_elapsed_seconds >= (state->current_green_seconds + TRAFFIC_YELLOW_SECONDS)) {
        phase_after_rr = next_phase_round_robin(state->current_phase);
        state->current_phase = select_highest_demand_phase(
            lane1,
            lane2,
            lane3,
            lane4,
            phase_after_rr);
        state->phase_elapsed_seconds = 0.0f;
        state->current_stage = TRAFFIC_STAGE_GREEN;
    }
}

void traffic_controller_get_outputs(const traffic_controller_state_t *state,
                                    traffic_signal_outputs_t *outputs) {
    if (state == 0 || outputs == 0) {
        return;
    }

    outputs->lane1_red = 1;
    outputs->lane1_yellow = 0;
    outputs->lane1_green = 0;
    outputs->lane2_red = 1;
    outputs->lane2_yellow = 0;
    outputs->lane2_green = 0;
    outputs->lane3_red = 1;
    outputs->lane3_yellow = 0;
    outputs->lane3_green = 0;
    outputs->lane4_red = 1;
    outputs->lane4_yellow = 0;
    outputs->lane4_green = 0;

    if (state->current_phase == TRAFFIC_PHASE_LANE1) {
        outputs->lane1_red = 0;
        if (state->current_stage == TRAFFIC_STAGE_GREEN) {
            outputs->lane1_green = 1;
        } else {
            outputs->lane1_yellow = 1;
        }
    } else if (state->current_phase == TRAFFIC_PHASE_LANE2) {
        outputs->lane2_red = 0;
        if (state->current_stage == TRAFFIC_STAGE_GREEN) {
            outputs->lane2_green = 1;
        } else {
            outputs->lane2_yellow = 1;
        }
    } else if (state->current_phase == TRAFFIC_PHASE_LANE3) {
        outputs->lane3_red = 0;
        if (state->current_stage == TRAFFIC_STAGE_GREEN) {
            outputs->lane3_green = 1;
        } else {
            outputs->lane3_yellow = 1;
        }
    } else {
        outputs->lane4_red = 0;
        if (state->current_stage == TRAFFIC_STAGE_GREEN) {
            outputs->lane4_green = 1;
        } else {
            outputs->lane4_yellow = 1;
        }
    }
}

float traffic_controller_get_current_green_seconds(const traffic_controller_state_t *state) {
    if (state == 0) {
        return TRAFFIC_DEFAULT_GREEN_SECONDS;
    }
    return state->current_green_seconds;
}

traffic_phase_t traffic_controller_get_current_phase(const traffic_controller_state_t *state) {
    if (state == 0) {
        return TRAFFIC_PHASE_LANE1;
    }
    return state->current_phase;
}

/* Convert green duration to timer ticks for TIMx->ARR style programming. */
uint32_t traffic_controller_seconds_to_ticks(float seconds, uint32_t timer_tick_hz) {
    float safe_seconds = clamp_green_seconds(seconds);
    if (timer_tick_hz == 0U) {
        return 0U;
    }
    return (uint32_t)(safe_seconds * (float)timer_tick_hz);
}
