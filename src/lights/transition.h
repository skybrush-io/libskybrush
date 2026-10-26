/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2026 CollMot Robotics Ltd.
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
 * \brief Encapsulates information about a linear color transition in progress on a LED strip.
 *
 * \tparam  TransitionHandler  a function that will be called with a single floating-point
 *             value between 0 and 1 from the \c step() method. The value
 *             corresponds to the progress of the transition.
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

public:
    /**
     * Constructor. Creates an inactive transition.
     */
    explicit Transition()
        : m_active(false)
        , m_start(0)
        , m_duration(0)
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
     * \brief Returns the progress of the transition.
     *
     * \param  clock  the value of the internal clock
     * \return the progress of the transition expressed as a value between 0 and 1.
     */
    transition_progress_t progress(unsigned long clock) const
    {
        transition_progress_t result;

        /* Defensive checks; the executor never steps a transition before its
         * start time or after its end time */
        if (clock < m_start) { /* LCOV_EXCL_BR_LINE */
            return 0; /* LCOV_EXCL_LINE */
        } else if (m_duration == 0) {
            return 1;
        } else {
            clock -= m_start;
            result = ((transition_progress_t)clock) / m_duration;
            return result > 1 ? 1 : result; /* LCOV_EXCL_BR_LINE */
        }
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
     * \param  handler  the transition handler to call with the progress of
     *                  the transition
     * \param  clock  the time on the internal clock
     * \return \c true if the transition shall continue, \c false if
     *         it has ended.
     */
    bool step(const TransitionHandler& handler, unsigned long clock)
    {
        transition_progress_t currentProgress = progress(clock);

        handler(currentProgress);

        m_active = currentProgress < 1;
        return m_active;
    }
};

#endif
