/*
 * This file is part of libskybrush.
 *
 * Copyright 2020-2022 by libskybrush authors. See AUTHORS.
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

#include "error_handler.h"

/**
 * Base error handler that does nothing.
 */
class DoNothingErrorHandler : public ErrorHandler {
public:
    DoNothingErrorHandler()
        : ErrorHandler() {};
    virtual void handleError(Errors::Code code) override {};

private:
    /**
     * Copy constructor. Intentionally left unimplemented to prevent the user
     * from copying the singleton instance.
     */
    DoNothingErrorHandler(DoNothingErrorHandler const&);

    /**
     * Assignment operator. Intentionally left unimplemented to prevent the
     * user from copying the singleton instance.
     */
    void operator=(DoNothingErrorHandler const&);
};

static DoNothingErrorHandler doNothingErrorHandler;
static ErrorHandler* errorHandler = &doNothingErrorHandler;

ErrorHandler* getErrorHandler()
{
    return errorHandler;
}

void setErrorHandler(ErrorHandler* handler)
{
    errorHandler = handler;
}
