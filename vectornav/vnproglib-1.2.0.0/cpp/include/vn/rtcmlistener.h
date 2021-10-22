#ifndef _VN_XPLAT_RTCMLISTENER_H_
#define _VN_XPLAT_RTCMLISTENER_H_

#include "rtcmmessage.h"
#include "export.h"

#include <functional>		// Used for the call back function
#include <string.h>

#ifdef _WIN32
#ifndef _WIN32_WINNT
	#define _WIN32_WINNT 0x0501
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else 
#include <sys/socket.h>		// Use POSIX-style sockets
#include <arpa/inet.h>
#include <netdb.h>			// Needed for getaddrinfo() and freeaddrinfo()
#include <unistd.h>			// Needed for close()
//#include <sys/types.h>
#endif

#include "thread.h"

namespace vn {
	namespace xplat {

		/// \brief Helpful class for working with RTCM broadcasters.
		class  vn_proglib_DLLEXPORT rtcmlistener
		{
		public:
			rtcmlistener();

			/// \brief Connect to the broadcasting server
			///
			/// \return bool					True if successful, otherwise false.
			bool connectToServer();

			/// \brief Disconnect from the broadcasting server
			///
			/// \return bool					True if successful, otherwise false.
			bool disconnectFromServer();


			/// \brief Register a callback funtion to be notified when a message arrives
			///
			/// \param[in] notificationHandler	The callback function to register
			void registerNotifications(std::function<void(rtcmmessage)> notificationHandler);

			/// \brief The host address of the broadcasting server
			///
			/// \param[in] host					the IP of the host
			void setServerHost(std::string host);

			/// \brief Set the server port of the boardcasting server
			///
			/// \param[in] port					the port to assign
			void setServerPort(std::string  port);

			/// \brief Thread to processing incoming messages from a RTCM broadcast server (used by the class when connecting to the server)
			void communicationThreadWithRTCMServer();




		private:
#ifdef _WIN32
			#define GETSOCKETERROR() (WSAGetLastError())
#else
			#define GETSOCKETERROR() (errno)
			#define NO_ERROR 0
			#define INVALID_SOCKET -1
			#define SOCKET_ERROR -1
			typedef int SOCKET;
#endif
			int sockInit();
			int sockQuit();
			int sockClose(SOCKET s);
			bool sockValid(SOCKET s);
			void printBuffer(char* buffer, int bufferSize);

			bool timeToQuit = false;
			std::string serverHost = "localhost";
			std::string serverPort = "8678";

			// POSIX-style sockets are int with negative values being invalid, while WinSock sockets are unsigned int with a special INVALID_SOCKET instead.
			SOCKET serverSocket = INVALID_SOCKET;

			//std::unique_ptr<std::thread> serverThread = nullptr;
			Thread* serverThread = nullptr;
			std::function<void(rtcmmessage)> messageNotification;
			bool messageNotificationActive = false;

		};

	}

}

#endif
