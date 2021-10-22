#include "vn/rtcmlistener.h"
#include "vn/thread.h"

#ifdef _WIN32
#pragma comment (lib, "Ws2_32.lib")
#endif
namespace vn {
	namespace xplat {

		static void HandleIncomingRTCMPackets(void* data)
		{
			((rtcmlistener*)data)->communicationThreadWithRTCMServer();
		}


		rtcmlistener::rtcmlistener()
		{
		}


		bool rtcmlistener::connectToServer()
		{
			// Start worker thread
			if (!serverThread)
			{
				//serverThread = std::unique_ptr<std::thread>(new std::thread(&RTCM_Server::CommunicationThreadWithRTCMServer, this));
				serverThread = Thread::startNew(HandleIncomingRTCMPackets, this);
			}

			return true;
		}


		bool rtcmlistener::disconnectFromServer()
		{
			bool result = false;

			timeToQuit = true;
			Thread::sleepSec(1);
			serverThread->join();
			serverThread = nullptr;

			printf("RTCM_Server::CommunicationThreadWithRTCMServer: Server Thread closed.\n");
			return result;
		}

		void rtcmlistener::communicationThreadWithRTCMServer()
		{
			// Initialize socket
			int iResult = sockInit();
			if (iResult != NO_ERROR)
			{
				printf("RTCM_Server::CommunicationThreadWithRTCMServer: sockInit failed with error: %d\n", iResult);
				return;
			}

			struct addrinfo* result = NULL;
			struct addrinfo* ptr = NULL;
			struct addrinfo hints;

			memset(&hints, 0, sizeof(hints));
			hints.ai_family = AF_INET;
			hints.ai_socktype = SOCK_STREAM;
			hints.ai_protocol = IPPROTO_IP;

			iResult = getaddrinfo(serverHost.c_str(), serverPort.c_str(), &hints, &result);
			if (iResult != 0)
			{
				printf("RTCM_Server::CommunicationThreadWithRTCMServer: getaddrinfo failed with error: %d\n", iResult);
				sockQuit();
				return;
			}

			// Attempt to connect to an address until one succeeds
			for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
			{
				// create a socket for connecting to server
				serverSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
				if (!sockValid(serverSocket))
				{
					printf("RTCM_Server::CommunicationThreadWithRTCMServer: socket failed with error: %d\n", GETSOCKETERROR());
					sockQuit();
					return;
				}

				// connect to server.
				iResult = connect(serverSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
				if (iResult == SOCKET_ERROR)
				{
					sockClose(serverSocket);
					serverSocket = INVALID_SOCKET;
					continue;
				}
				break;
			}

			freeaddrinfo(result);

			if (!sockValid(serverSocket))
			{
				printf("RTCM_Server::CommunicationThreadWithRTCMServer: Unable to connect to server!\n");
				sockQuit();
				return;
			}

			int bytesReceived = 0;
			do {
#define DEFAULT_BUFLEN 10240
				char buffer[DEFAULT_BUFLEN];
				int bufferLen = DEFAULT_BUFLEN;

				//printf("RTCM_Server::CommunicationThreadWithRTCMServer: Receive data from server.\n");
				bytesReceived = recv(serverSocket, buffer, bufferLen, 0);
				//printBuffer(buffer, bytesReceived);
				if (bytesReceived > 0)
				{
					int index = 0;

					// Separate out all the messages from the buffer
					while (index < bytesReceived - 2)
					{
						// Verify packet signature (0xD3) before attempting to process
						if (buffer[index] == (char)0xD3)
						{
							// Ignore the first 6 bits of the second byte when calculating the size
							int packetSize = (((buffer[index + 1] & 0xFF) & 0x03) * 256) + (buffer[index + 2] & 0xFF) + 6;
							// NOTE: Need to add some checks just in case the sync byte is the very last byte in the buffer
							//       May need to do another read to get the rest of the message

							// Sanity check on packetSize is not greater than remaining buffer size
							if (packetSize + index > bytesReceived)
							{
								// Dang it, something is not right and will cause problems if we try to process this. Just skip it and keep looking.
								index++;
							}
							else
							{
								rtcmmessage message = rtcmmessage(buffer, bytesReceived, index, packetSize);
								// Only keep valid messages that the sensor will accept

								if (message.valid)
								{
									index += packetSize;

									if (message.supported)
									{
										//printf("RTCM_Server::CommunicationThreadWithRTCMServer: Message %d received.\n", message.id);
										if (messageNotificationActive)
										{
											messageNotification(message);
										}
									}
									else
									{
										// If this message is not supported, then just drop it.
										//printf("RTCM_Server::CommunicationThreadWithRTCMServer: Message %d is not supported.\n", message.id);
									}
								}
								else
								{
									// If the message is invalid, chances are we are in the middle of a packet and just need to find the start of the next message
									//printf("RTCM_Server::CommunicationThreadWithRTCMServer: Invalid Message %d Found.\n", message.id);
									index++;
								}
							}
						}
						else
						{
							// Keep working our way down the buffer until we find a message header
							index++;
						}
					}
					//printf("Finished parsing buffer.\n");
				}
				else if (bytesReceived == 0)
				{
					//printf("Connection closed.\n");
				}
				else
				{
					printf("recv failed with error: %d\n", GETSOCKETERROR());
				}
			} while ((bytesReceived > 0) && (!timeToQuit));

			sockClose(serverSocket);
			sockQuit();
		}


		void rtcmlistener::registerNotifications(std::function<void(rtcmmessage)> notificationHandler)
		{
			messageNotification = notificationHandler;
			messageNotificationActive = true;
		}

		void rtcmlistener::setServerHost(std::string host)
		{
			serverHost = host;
		}

		void rtcmlistener::setServerPort(std::string  port)
		{
			serverPort = port;
		}


		int rtcmlistener::sockInit()
		{
#ifdef _WIN32
			WSADATA wsa_data;
			return WSAStartup(MAKEWORD(1, 1), &wsa_data);
#else
			return 0;
#endif
		}

		int rtcmlistener::sockQuit()
		{
#ifdef _WIN32
			return WSACleanup();
#else
			return 0;
#endif
		}

		int rtcmlistener::sockClose(SOCKET s)
		{
			int status = 0;

#ifdef _WIN32
			status = shutdown(s, SD_BOTH);
			if (status == 0)
			{
				status = closesocket(s);
			}
#else
			status = shutdown(s, SHUT_RDWR);
			if (status == 0)
			{
				status = close(s);
			}
#endif

			return status;
		}

		bool rtcmlistener::sockValid(SOCKET s)
		{
			bool status = false;

#ifdef _WIN32
			status = s != INVALID_SOCKET;
#else
			if (s > 0)
			{
				status = true;
			}
#endif

			return status;
		}

		void rtcmlistener::printBuffer(char* buffer, int bufferSize)
		{
			printf("BUFFER[");
			for (int i = 0; i < bufferSize; i++)
			{
				printf("0x%02hhx, ", buffer[i] & 0xFF);
			}
			printf("]\n");
		}
	}
}