/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2024 CollMot Robotics Ltd.
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
 * \file src/lights/error_handler.h
 * Interface specification for objects that are used to handle error conditions
 * signalled from the light player part of the library.
 */

#ifndef SKYBRUSH_LIGHTS_ERROR_HANDLER_H
#define SKYBRUSH_LIGHTS_ERROR_HANDLER_H

#include "errors.h"

/**
 * \brief Interface specification for objects that are used to handle error conditions
 * signalled from the light player part of the library.
 */
class ErrorHandler {
protected:
    /**
     * Code of the last error that happened during execution.
     */
    Errors::Code m_error;

public:
    /**
     * Constructor. This should not be called directly as we don't want the user to
     * instantiate this object.
     */
    ErrorHandler()
        : m_error(Errors::SUCCESS)
    {
    }

    /**
     * Tells the error handler that there is no error condition at the moment.
     */
    void clearError()
    {
        setError(Errors::SUCCESS);
    }

    /**
     * Asks the error handler to set a new error code.
     *
     * The implementation of this function will call \c handleError() if the
     * error code is different from the previous one.
     */
    void setError(Errors::Code code)
    {
        if (m_error != code) {
            m_error = code;
            handleError(code);
        }
    }

protected:
    /**
     * Asks the error handler to handle the given error condition.
     *
     * \param  code  the code of the error
     */
    virtual void handleError(Errors::Code code) = 0;
};

/**
 * Returns a pointer to the current instance of the error handler.
 */
ErrorHandler* getErrorHandler();

/**
 * Sets a new error handler.
 */
void setErrorHandler(ErrorHandler* handler);

/**
 * \def SET_ERROR
 *
 * Shorthand notation for setting an error code in the current error handler.
 */
#define SET_ERROR(code) getErrorHandler()->setError(code)

/**
 * \def CLEAR_ERROR
 *
 * Shorthand notation for clearing the error condition in the current error handler.
 */
#define CLEAR_ERROR getErrorHandler()->clearError

#endif
