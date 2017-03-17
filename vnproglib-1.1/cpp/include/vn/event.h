#ifndef _VNXPLAT_EVENT_H_
#define _VNXPLAT_EVENT_H_

#include "nocopy.h"
#include "int.h"
#include "export.h"

namespace vn {
namespace xplat {

/// \brief Represents a cross-platform event.
class vn_proglib_DLLEXPORT Event : private util::NoCopy
{

public:

	/// \brief Available wait results.
	enum WaitResult
	{
		WAIT_SIGNALED,	///< The event was signalled.
		WAIT_TIMEDOUT	///< Timed out while waiting for the event to signal.
	};

	// Constructors ///////////////////////////////////////////////////////////

public:

	/// \brief Creates a new event.
	Event();

	~Event();

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Waits for a signal on this event.
	///
	/// This method will wait indefinitely for the event.
	void wait();

	/// \brief Waits for a signal on this event for the specified amount of
	/// time.
	///
	/// \param[in] timeoutUs The amount of time to wait in microseconds.
	/// \return The result of the wait operation.
	WaitResult waitUs(uint32_t timeoutUs);

	/// \brief Waits for a signal on this event for the specified amount of
	/// time.
	///
	/// \param[in] timeoutMs The amount of time to wait in milliseconds.
	/// \return The result of the wait operation.
	WaitResult waitMs(uint32_t timeoutMs);

	/// \brief Signals the event.
	void signal();

	// Private Members ////////////////////////////////////////////////////////

private:

	// Contains internal data, mainly stuff that is required for cross-platform
	// support.
	struct Impl;
	Impl *_pi;

};

}
}

#endif
