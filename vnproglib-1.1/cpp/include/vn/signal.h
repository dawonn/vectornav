#ifndef _VNXPLAT_SIGNAL_H_
#define _VNXPLAT_SIGNAL_H_

namespace vn {
namespace xplat {

/// \brief Provides access to system signals.
class Signal
{
	// Types //////////////////////////////////////////////////////////////////

public:

	/// \brief The available signals.
	enum SignalType
	{
		/// Unknown type of signal.
		UNKNOWN,

		/// User pressed <em>Ctrl-C</em>.
		CTRL_C
	};

	/// \brief Typedef for a function that can handle signal notifications.
	///
	/// \param[in] signal The signal type received.
	/// \return Indicates if the signal was handled or not.
	typedef bool (*HandleSignalFunc)(Signal signal);

	/// \brief Allows for other objects to listen for signal events.
	class Observer
	{
	public:
		virtual ~Observer() { }

		/// \brief If an observer is registered via
		///     \ref RegisterSignalObserver, whenever a signal is received,
		///     this method will be called to alert the observer.
		///
		/// \param[in] signal The signal that was raised.
		/// \return Indicates if the signal was handled or not.
		virtual bool XSignalHandleSingle(SignalType signal) = 0;

	protected:
		Observer() { }
	};

	// Public Methods /////////////////////////////////////////////////////////

public:

	/// \brief Allows registering to receive notifications of system signals.
	///
	/// \param[in] handleFunc Function to call when signals are received.
	static void RegisterForSignalNotifications(
		HandleSignalFunc handleFunc);

	/// \brief Allows unregistering from receiving signal notifications.
	///
	/// \param[in] handleFunc The function to unregister.
	static void UnregisterForSignalNotifications(
		HandleSignalFunc handleFunc);

	/// \brief Allows registering an observer for notification when a signal is
	///     received.
	///
	/// \param[in] observer The observer to register.
	static void RegisterSignalObserver(
		Observer* observer);

	/// \brief Allows unregistering of an observer from being notified when a
	///     signal is received.
	///
	/// \param[in] observer The observer to unregister.
	static void UnregisterSignalObserver(
		Observer* observer);

	// Private Members ////////////////////////////////////////////////////////

private:

	// Contains internal data, mainly stuff that is required for cross-platform
	// support.
	struct Impl;

};

}
}

#endif
