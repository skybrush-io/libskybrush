/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2025 CollMot Robotics Ltd.
 *
 * libskybrush is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * libskybrush is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * \file src/lights/transition.h
 * \brief Color transition handling on a LED strip.
 */

#ifndef SKYBRUSH_LIGHTS_TRANSITION_H
#define SKYBRUSH_LIGHTS_TRANSITION_H

/**
 * \brief Defines the floating-point type that transitions will use.
 *
 * Switch this to \c double for extra precision (though this should not
 * be needed).
 */
typedef float transition_progress_t;

/**
 * \brief Easing modes for transitions.
 *
 * See http://easings.net for more information about the modes being used here.
 */
typedef enum {
    EASING_LINEAR,
    EASING_IN_SINE,
    EASING_OUT_SINE,
    EASING_IN_OUT_SINE,
    EASING_IN_QUAD,
    EASING_OUT_QUAD,
    EASING_IN_OUT_QUAD,
    EASING_IN_CUBIC,
    EASING_OUT_CUBIC,
    EASING_IN_OUT_CUBIC,
    EASING_IN_QUART,
    EASING_OUT_QUART,
    EASING_IN_OUT_QUART,
    EASING_IN_QUINT,
    EASING_OUT_QUINT,
    EASING_IN_OUT_QUINT,
    EASING_IN_EXPO,
    EASING_OUT_EXPO,
    EASING_IN_OUT_EXPO,
    EASING_IN_CIRC,
    EASING_OUT_CIRC,
    EASING_IN_OUT_CIRC,
    EASING_IN_BACK,
    EASING_OUT_BACK,
    EASING_IN_OUT_BACK,
    EASING_IN_ELASTIC,
    EASING_OUT_ELASTIC,
    EASING_IN_OUT_ELASTIC,
    EASING_IN_BOUNCE,
    EASING_OUT_BOUNCE,
    EASING_IN_OUT_BOUNCE,
    NUM_EASING_FUNCTIONS
} EasingMode;

/**
 * \brief Typedef for easing functions.
 */
typedef transition_progress_t easing_function_t(transition_progress_t);

/**
 * \brief Mapping from easing modes to the corresponding easing functions.
 */
extern easing_function_t* const EASING_FUNCTIONS[NUM_EASING_FUNCTIONS];

/**
 * \brief Encapsulates information about a color transition in progress on a LED strip.
 *
 * \tparam  TransitionHandler  a function that will be called with a single floating-point
 *             value between 0 and 1 from the \c step() method. The value
 *             corresponds to the progress of the transition after the
 *             easing function has been applied.
 */
template <typename TransitionHandler>
class Transition {
private:
    /**
     * Whether the transition is currently active.
     */
    bool m_active;

    /**
     * The start time of the transition.
     */
    unsigned long m_start;

    /**
     * The duration of the transition.
     */
    unsigned long m_duration;

    /**
     * The easing mode of the transition.
     */
    EasingMode m_easingMode;

public:
    /**
     * Constructor. Creates an inactive transition.
     */
    explicit Transition()
        : m_active(false)
        , m_start(0)
        , m_duration(0)
        , m_easingMode(EASING_LINEAR)
    {
    }

    /**
     * Returns whether the transition is currently active.
     */
    bool active() const
    {
        return m_active;
    }

    /**
     * Cancels the current transition immediately by setting its "active" flag to false.
     */
    void cancel()
    {
        m_active = false;
    }

    /**
     * Returns the current easing mode of the transition.
     */
    EasingMode easingMode() const
    {
        return m_easingMode;
    }

    /**
     * \brief Returns the progress of the transition \em before applying the easing function.
     *
     * \param  clock  the value of the internal clock
     * \return the progress of the transition expressed as a value between 0 and 1.
     */
    transition_progress_t progressPreEasing(unsigned long clock) const
    {
        transition_progress_t result;

        if (clock < m_start) {
            return 0;
        } else if (m_duration == 0) {
            return 1;
        } else {
            clock -= m_start;
            result = ((transition_progress_t)clock) / m_duration;
            return result > 1 ? 1 : result;
        }
    }

    /**
     * \brief Returns the progress of the transition \em after applying the easing function.
     *
     * Note that for some easing functions, the progress value can be negative or larger
     * than 1 for certain times.
     *
     * \param  clock  the value of the internal clock
     */
    transition_progress_t progressPostEasing(unsigned long clock) const
    {
        return EASING_FUNCTIONS[m_easingMode](progressPreEasing(clock));
    }

    /**
     * Sets the current easing mode of the transition.
     */
    void setEasingMode(EasingMode value)
    {
        m_easingMode = value;
    }

    /**
     * Starts a transition with the given duration and start time.
     *
     * \param  duration   the duration of the transition
     * \param  startTime  the start time of the transition.
     */
    void start(unsigned long duration, unsigned long startTime)
    {
        m_start = startTime;
        m_duration = duration;
        m_active = true;
    }

    /**
     * Makes a step in the transition, assuming that the internal
     * clock is at the given time.
     *
     * \param  handler  the transition handler to call with the post-easing
     *                  progress of the transition
     * \param  clock  the time on the internal clock
     * \return \c true if the transition shall continue, \c false if
     *         it has ended.
     */
    bool step(const TransitionHandler& handler, unsigned long clock)
    {
        transition_progress_t progress = progressPreEasing(clock);
        transition_progress_t transformedProgress = EASING_FUNCTIONS[m_easingMode](progress);

        handler(transformedProgress);

        m_active = progress < 1;
        return m_active;
    }
};

#endif
