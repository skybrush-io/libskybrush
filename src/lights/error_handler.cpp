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
