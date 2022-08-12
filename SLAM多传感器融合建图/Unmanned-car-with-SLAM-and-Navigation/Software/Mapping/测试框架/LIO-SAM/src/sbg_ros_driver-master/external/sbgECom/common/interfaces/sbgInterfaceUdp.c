#include "sbgInterfaceUdp.h"

#ifdef WIN32
	#include <winsock2.h>
	#include <WS2tcpip.h>
	#include <stdint.h>
#else
	#include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <unistd.h>
    #include <fcntl.h>
	
	
	#define SOCKADDR_IN    struct sockaddr_in
	#define SOCKADDR       struct sockaddr
	#define SOCKET         int
	#define INVALID_SOCKET (SOCKET)(~0)
	#define SOCKET_ERROR   (-1)
	#define NO_ERROR       (0)
	#define SD_BOTH         (2)
	
	#define closesocket(socket)    close(socket); 
	
#endif

//----------------------------------------------------------------------//
//- Private methods declarations                                       -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the socket API.
 *	\return										SBG_NO_ERROR if the socket API has been correctly initialized.
 */
SbgErrorCode sbgInterfaceUdpInitSockets(void)
{
#ifdef WIN32
		WSADATA wsaData;

		//
		// Initialize windows sockets version 2.2
		//
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) == NO_ERROR)
		{
			return SBG_NO_ERROR;
		}
		else
		{
			return SBG_ERROR;
		}
#else
	//
	// For Unix platform, the socket API doesn't requiere any initialization
	//
	return	SBG_NO_ERROR;
#endif
}

/*!
 *	Uninitialize the socket API.
 *	\return										SBG_NO_ERROR if the socket API has been uninitialized.
 */
SbgErrorCode sbgInterfaceUpdateCloseSockets(void)
{
#ifdef WIN32
		//
		// Release windows sockets
		//
		if (WSACleanup() == NO_ERROR)
		{
			return SBG_NO_ERROR;
		}
		else
		{
			return SBG_ERROR;
		}
#else
	//
	// For Unix platform, the socket API doesn't requiere any initialization
	//
	return	SBG_NO_ERROR;
#endif		
}

/*!
 *	Define if a socket should block or not on receive and send calls.
 *	\param[in]	socketHandle					The socket handle to change.
 *	\param[in]	blocking						Set to true for a blocking socket or false for a non blocking socket.
 *	\return										SBG_NO_ERROR if the blocking status has been changed.
 */
SbgErrorCode sbgInterfaceUdpSetSocketBlocking(SOCKET socketHandle, bool blocking)
{
#ifdef WIN32
	u_long blockingMode;

	//
	// Define if we would like a blocking (0 or non blocking socket 1)
	//
	blockingMode = (blocking ?0:1);

	//
	// Define the socket as non blocking
	//
	if (ioctlsocket(socketHandle, FIONBIO, &blockingMode) == NO_ERROR)
	{
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_ERROR;
	}
#else
	int32 flags;
	
	//
	// Get the current socket flags and options
	//
	flags = fcntl(socketHandle, F_GETFL, 0);

	//
	// Make sure that there is no error (return value should be positive or zero)
	//
	if (flags >= 0)
	{
		//
		// Update the flags to enable or disable the blocking mode
		//
		flags = (blocking ? (flags&~O_NONBLOCK) : (flags|O_NONBLOCK));

		//
		// Redefine the new flags
		//
		if (fcntl(socketHandle, F_SETFL, flags) == 0)
		{
			return SBG_NO_ERROR;
		}
	}
	
	return SBG_ERROR;
#endif
}

//----------------------------------------------------------------------//
//- Operations methods declarations                                    -//
//----------------------------------------------------------------------//

/*!
 *	Initialize an unconnected UDP interface for read and write operations.
 *	An UDP interface can send some data to an output ip address and port and read all received
 *	data on a input port.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	remoteAddr						IP address to send data to.
 *	\param[in]	remotePort						Ethernet port to send data to.
 *	\param[in]	localPort						Ehternet port on which the interface is listening.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SbgErrorCode sbgInterfaceUdpCreate(SbgInterface *pHandle, sbgIpAddress remoteAddr, uint32 remotePort, uint32 localPort)
{
	SbgErrorCode		 errorCode = SBG_NO_ERROR;
	SbgInterfaceUdp		*pNewUdpHandle;
	SOCKADDR_IN			 bindAddress;
	SOCKET				 newUdpSocket;
	
	assert(pHandle);

	//
	// Initialize the socket API
	//
	if (sbgInterfaceUdpInitSockets() == SBG_NO_ERROR)
	{
		//
		// Create an UDP handle
		//
		pNewUdpHandle = (SbgInterfaceUdp*)malloc(sizeof(SbgInterfaceUdp));

		//
		// Test that the UDP handle has been allocated
		//
		if (pNewUdpHandle)
		{
			//
			// Fill the UDP handle
			//
			pNewUdpHandle->remoteAddr	= remoteAddr;
			pNewUdpHandle->remotePort	= remotePort;
			pNewUdpHandle->localPort	= localPort;

			//
			// Allocate a socket for windows we do this because we don't know the socket type and we would like
			// a base and common interface for both windows or Unix platforms.
			//
			pNewUdpHandle->pUdpSocket = (SOCKET*)malloc(sizeof(SOCKET));
			
			//
			// Make sure that the socket has been created
			//
			if (pNewUdpHandle->pUdpSocket)
			{
				//
				// Create a socket to send and/or receive UDP data for IPv4 addresses
				//
				newUdpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

				//
				// Test that the socket has been created
				//
				if (newUdpSocket != INVALID_SOCKET)
				{
					//
					// Define the socket as non blocking
					//
					if (sbgInterfaceUdpSetSocketBlocking(newUdpSocket, false) == NO_ERROR)
					{
						//
						// The bind should accept a connection on any ip address but only on the input port
						//
						bindAddress.sin_family = AF_INET;
						bindAddress.sin_addr.s_addr = INADDR_ANY;
						bindAddress.sin_port = htons((uint16_t)localPort);

						//
						// Bind this socket to all ip addresses on the input port
						//
						if (bind(newUdpSocket, (SOCKADDR*) &bindAddress, sizeof(bindAddress)) != SOCKET_ERROR)
						{
							//
							// Fill the created UDP socket
							//
							*((SOCKET*)pNewUdpHandle->pUdpSocket) = newUdpSocket;
									
							//
							// The UDP interface is ready so fill the generic interface
							//
							pHandle->handle = pNewUdpHandle;
							pHandle->type = SBG_IF_TYPE_ETH_UDP;
							pHandle->pReadFunc = sbgInterfaceUdpRead;
							pHandle->pWriteFunc = sbgInterfaceUdpWrite;

							//
							// Return without any error
							//
							return SBG_NO_ERROR;							
						}
						else
						{
							//
							// Unable to bind the address
							//
							errorCode = SBG_ERROR;
						}
					}
					else
					{
						//
						// Unable to define the non blocking mode
						//
						errorCode = SBG_ERROR;
					}

					//
					// Close the UDP socket
					//
					shutdown(newUdpSocket, SD_BOTH);
					closesocket(newUdpSocket);
				}
				else
				{
					//
					// Unable to create the socket
					//
					errorCode = SBG_ERROR;
				}

				//
				// If needed, free the created socket
				//
				SBG_FREE(pNewUdpHandle->pUdpSocket);
			}
			else
			{
				//
				// Unable to allocate memory for the UDP socket
				//
				errorCode = SBG_MALLOC_FAILED;
			}

			//
			// Free the UDP handle
			//
			SBG_FREE(pNewUdpHandle);
		}
		else
		{
			//
			// Allocation failed
			//
			errorCode = SBG_MALLOC_FAILED;
		}

		//
		// Close the sockets API
		//
		sbgInterfaceUpdateCloseSockets();
	}

	return errorCode;
}

/*!
 *	Destroy an interface initialized using sbgInterfaceUdpCreate.
 *	\param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SbgErrorCode sbgInterfaceUdpDestroy(SbgInterface *pHandle)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgInterfaceUdp *pUdpHandle;
	SOCKET udpSocket;

	assert(pHandle);
	assert(pHandle->type == SBG_IF_TYPE_ETH_UDP);

	//
	// Get the UDP handle
	//
	pUdpHandle = (SbgInterfaceUdp*)pHandle->handle;

	//
	// Test if we have to close the UDP socket
	//
	if (pUdpHandle->pUdpSocket)
	{
		//
		// Get the UDP receive socket
		//
		udpSocket = *((SOCKET*)pUdpHandle->pUdpSocket);

		//
		// Close the UDP receive socket
		//
		shutdown(udpSocket, SD_BOTH);
		closesocket(udpSocket);

		//
		// Release the UDP receive socket
		//
		SBG_FREE(pUdpHandle->pUdpSocket);
	}
		
	//
	// Release the UDP interface
	//
	SBG_FREE(pUdpHandle);

	//
	// Close the socket API
	//
	errorCode = sbgInterfaceUpdateCloseSockets();

	return errorCode;
}

/*!
 *	Define if a socket can send broadcasted packets.
 *	\param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 *	\param[in]	allowBroadcast					Set to true to allow this socket to send broadcasted UDP packets.
 *	\return										SBG_NO_ERROR if the allow broadcast status has been changed.
 */
SbgErrorCode sbgInterfaceUdpAllowBroadcast(SbgInterface *pHandle, bool allowBroadcast)
{
	SbgErrorCode			 errorCode = SBG_NO_ERROR;
	SbgInterfaceUdp			*pUdpHandle;
	SOCKET					 udpSocket;
	uint32					 broadcastMode;

	assert(pHandle);
	assert(pHandle->type == SBG_IF_TYPE_ETH_UDP);
		
	//
	// Get the UDP handle
	//
	pUdpHandle = (SbgInterfaceUdp*)pHandle->handle;

	//
	// Test that we have a valid UDP send socket
	//
	if (pUdpHandle->pUdpSocket)
	{
		//
		// Get the UDP send socket
		//
		udpSocket = *((SOCKET*)pUdpHandle->pUdpSocket);

		//
		// Define if we would like to enable UDP broadcast (0 No, 1 Yes)
		//
		broadcastMode = (allowBroadcast?true:false);

		//
		// Define if the socket can send broadcasted packets
		//
		if (setsockopt(udpSocket, SOL_SOCKET, SO_BROADCAST, (const char*)&broadcastMode, sizeof(broadcastMode)) != NO_ERROR)
		{
			//
			// Unable to define socket options
			//
			errorCode = SBG_ERROR;
		}
	}
	else
	{
		//
		// No valid send UDP socket found
		//
		errorCode = SBG_NULL_POINTER;
	}	

	return errorCode;
}

//----------------------------------------------------------------------//
//- Internal interfaces write/read implementations                     -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SbgErrorCode sbgInterfaceUdpWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgInterfaceUdp *pUdpHandle;
	SOCKADDR_IN outAddr;
	int numBytesSent;
	int partialWriteSize;
	SOCKET udpSocket;

	assert(pHandle);
	assert(pHandle->type == SBG_IF_TYPE_ETH_UDP);
	assert(pBuffer);

	//
	// Get the UDP handle
	//
	pUdpHandle = (SbgInterfaceUdp*)pHandle->handle;

	//
	// Get the UDP socket
	//
	udpSocket = *((SOCKET*)pUdpHandle->pUdpSocket);

	//
	// Define the receiver address and port
	//
	outAddr.sin_family = AF_INET;
	outAddr.sin_addr.s_addr = pUdpHandle->remoteAddr;				// Warning: the sbgIpAddress value is always stored in network endianness (ie big endian)
	outAddr.sin_port = htons((uint16_t)pUdpHandle->remotePort);

	//
	// Send packets until no more to send
	//
	while (bytesToWrite)
	{
		//
		// Initialize number of bytes to write
		//
		partialWriteSize = (int)bytesToWrite;

		//
		// Test if this packet is not over the UDP packet size limit
		//
		if (partialWriteSize > SBG_INTERFACE_UDP_PACKET_MAX_SIZE)
		{
			//
			// Set it to the limit
			//
			partialWriteSize = SBG_INTERFACE_UDP_PACKET_MAX_SIZE;
		}

		//
		// Send the datagram to the receiver
		//
		numBytesSent = sendto(udpSocket, (const char*)pBuffer, partialWriteSize, 0, (SOCKADDR*)&outAddr, sizeof(outAddr));

		//
		// Test that all the bytes have been written
		//
		if (numBytesSent != partialWriteSize)
		{
			//
			// The buffer has not been written successfully
			//
			break;
		}

		//
		// Update number of bytes to write and source buffer address
		//
		bytesToWrite -= (size_t)partialWriteSize;
		pBuffer = (uint8*)pBuffer + partialWriteSize;
	}

	//
	// Test that all the bytes have been written
	//
	if (bytesToWrite)
	{
		//
		// Unable to write some bytes
		//
		errorCode = SBG_WRITE_ERROR;
	}

	return errorCode;
}

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SbgErrorCode sbgInterfaceUdpRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgInterfaceUdp *pUdpHandle;
	int retValue;
	SOCKET udpSocket;

	assert(pHandle);
	assert(pHandle->type == SBG_IF_TYPE_ETH_UDP);
	assert(pBuffer);
	assert(pReadBytes);

	//
	// Get the UDP handle
	//
	pUdpHandle = (SbgInterfaceUdp*)pHandle->handle;

	//
	// Get the UDP socket
	//
	udpSocket = *((SOCKET*)pUdpHandle->pUdpSocket);

	//
	// Send the datagram to the receiver
	//
	retValue = recvfrom(udpSocket, (char*)pBuffer, (int)bytesToRead, 0, NULL, 0);

	//
	// Test that we don't have any error
	//
	if (retValue != SOCKET_ERROR)
	{
		//
		// Returns the number of read bytes
		//
		*pReadBytes = (size_t)retValue;
	}
	else
	{
		#ifdef WIN32
		//
		// Get the last error during the read
		//
		retValue = WSAGetLastError();

		if (retValue== WSAEWOULDBLOCK)
		{
			errorCode = SBG_NOT_READY;
		}
		else
		{
			errorCode = SBG_ERROR;
		}
		#else
		errorCode = SBG_NOT_READY;
		#endif
	}

	return errorCode;
}
