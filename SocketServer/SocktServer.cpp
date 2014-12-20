#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <unistd.h>
#include <asm/ioctls.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>

#include <queue>

#include "SocketServer.h"

using namespace std;


//*** local defines and typedefs **********************************************

#define SOCKET_POLLING_INTERVAL			(50 * 1000)	// uS

typedef struct
{
    // Info
	bool bStarted;
	U16	 u16Port;
    U16  u16NumClientsConnected;

    // Control
    bool bDisconnectAllClients;
    bool bStopServer;

} SOCKET_SERVER_INFO;

typedef struct
{
	// Info
	int 		socket_fd;		// opened socket file descriptor from the server thread
	U16			u16ClientId;	// Used as an ID for proper message routing
	bool		bConnected;		// Indicates if this client info is available for a new connection

	// Control
	bool 		bDisconnect;

	void        (*callbackFn)(int);

	// Data
    queue<U8> RxQueue;
    queue<U8> TxQueue;

} SOCKET_CLIENT_INFO;

//*** local function declarations *********************************************
static void *SocketServerThread( void *ptServerInfo );
static void *ClientThread( void *ptClientInfo );
SOCKET_CLIENT_INFO *GetMatchingClient( U16 eClientId );

//*** global variable definitions *********************************************
static SOCKET_SERVER_INFO gtSocketServerInfo;
static SOCKET_CLIENT_INFO gtClientInfo[SOCKET_MAX_CLIENT_CONNECTIONS];

//******************************************************************************
//
//	GetMatchingClient
//
//	This function returns the active client that matches the eClientId
//
//	Parameters:
//		eClientId: Client Id to match i.e. E_COMM_PORT_SOCKET_x
//
//	Returns:
//		SOCKET_CLIENT_INFO *: returns a pointer to the matching client, NULL otherwise
//
//******************************************************************************
SOCKET_CLIENT_INFO *GetMatchingClient( U16 eClientId )
{
	if( gtSocketServerInfo.bStarted && gtSocketServerInfo.u16NumClientsConnected > 0 )
	{
		// Find the client info that matches the ePortId
		int i;
		for( i=0; i<SOCKET_MAX_CLIENT_CONNECTIONS; i++ )
		{
			if( gtClientInfo[i].bConnected && ( eClientId == gtClientInfo[i].u16ClientId ))
			{
				return &gtClientInfo[i];
			}
		}
	}

	return NULL;
}

//******************************************************************************
//
//	Setup
//
//	This function initializes the SOCKET thread stuff
//
//	Parameters:
//		none
//
//	Returns:
//		nothing
//
//******************************************************************************
void Server_Init( void (*read_callback)(int) )
{
	int i;

	// Initialize the global client info structs
	for( i=0; i<SOCKET_MAX_CLIENT_CONNECTIONS; i++ )
	{
		gtClientInfo[i].bConnected = false;
		gtClientInfo[i].bDisconnect = false;
		gtClientInfo[i].socket_fd = -1;
		gtClientInfo[i].u16ClientId = i;
		gtClientInfo[i].callbackFn = read_callback;
	}
}

//******************************************************************************
//
//	StartServer
//
//	This function Starts the Socket Server
//
//	Parameters:
//		NONE
//
//	Returns:
//		false if fail, true otherwise
//
//******************************************************************************
bool Server_Start( U16 port )
{
	if( false == gtSocketServerInfo.bStarted )
	{
		gtSocketServerInfo.bStopServer = false;
		gtSocketServerInfo.u16Port = port;

		// Spawn the Server Thread that will listen for client connections
		pthread_t server_thread;

		if( pthread_create( &server_thread, NULL, SocketServerThread, (void*)&gtSocketServerInfo ) < 0 )
		{
			fprintf(stderr, "SOCKET: ERROR starting socket server\n");
			return false;
		}
	}
	else
	{
		printf("SOCKET: Socket Server already running\n");
	}

	return true;
}

//******************************************************************************
//
//	Stop
//
//	This function disconnects and shuts down the client sockets AND the server
//
//	Parameters:
//		NONE
//
//	Returns:
//		NONE
//
//******************************************************************************
void Server_Stop( void )
{
	gtSocketServerInfo.bStopServer = true;
}

//******************************************************************************
//
//	Server_GetRxData
//
//	This function is called by the upper-layer application to get bytes
//  received and stored by the client thread below
//
//	Parameters:
//		client: which client thread to read data from
//
//	Returns:
//		U16: number of bytes read from the socket client
//
//******************************************************************************
U16 Server_GetRxData( int client, U8 *u8RxBuffer )
{
    U16 i;
    U16 bytes_read;

    if( client > SOCKET_MAX_CLIENT_CONNECTIONS )
    {
        return 0;
    }

    bytes_read = gtClientInfo[client].RxQueue.size();

    for(i=0; i<bytes_read; i++)
    {
        u8RxBuffer[i] = gtClientInfo[client].RxQueue.front();
        gtClientInfo[client].RxQueue.pop();
    }

    return bytes_read;
}

//******************************************************************************
//
//	Server_SendTxData
//
//	This function is called by the upper-layer application to send bytes
//  to a connected client.
//
//	Parameters:
//		client: which client thread to write data to
//
//	Returns:
//		U16: number of bytes sent to the socket client
//
//******************************************************************************
U16 Server_SendTxData( int client, U8 *u8TxBuffer, U16 bytes )
{
    U16 i;

    if( client > SOCKET_MAX_CLIENT_CONNECTIONS )
    {
        return 0;
    }

    // store Tx data to the queue
    // bytes will be sent out by the client thread below
    for(i=0; i<bytes; i++)
    {
        gtClientInfo[client].TxQueue.push(u8TxBuffer[i]);
    }

    return i;
}

// ======================== Local Functions ====================================

//******************************************************************************
//
//	Disconnect
//
//	This function disconnects and shuts down the client sockets but leaves the
//  the server running. Use StopServer to shut it down.
//
//	Parameters:
//		NONE
//
//	Returns:
//		NONE
//
//******************************************************************************
void DisconnectClients( void )
{
	int i;

	if( gtSocketServerInfo.bStarted && gtSocketServerInfo.u16NumClientsConnected > 0 )
	{
		for( i=0; i<SOCKET_MAX_CLIENT_CONNECTIONS; i++ )
		{
			gtClientInfo[i].bDisconnect = true;
		}
	}

	gtSocketServerInfo.u16NumClientsConnected = 0;
}


//******************************************************************************
//
//	SocketServerThread
//
//	This is the server listening thread that waits for a client to connect
//
//	Parameters:
//		ptServerInfo - pointer the global SOCKET_SERVER_INFO
//
//	Returns:
//		NONE - check gtSocketServerInfo for status
//
//******************************************************************************
static void *SocketServerThread( void *ptServerInfo )
{
	SOCKET_SERVER_INFO *pServer = (SOCKET_SERVER_INFO*)ptServerInfo;
	int sockfd, flags, errno;
	int newsockfd, portno, i;
	socklen_t clilen;
	struct sockaddr_in serv_addr, cli_addr;

	// Setup server socket

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0)
	{
		fprintf(stderr, "SOCKET Server: socket Error: %s\n", strerror(errno));
		pthread_exit(0);
	}

	// Allow multiple sockets to use the same PORT number
	i = 1;
	if( setsockopt( sockfd, SOL_SOCKET, SO_REUSEADDR, &i, sizeof(i) ) < 0 )
	{
		fprintf(stderr, "SOCKET Server: setsockopt Error: %s\n", strerror(errno));
		pthread_exit(0);
	}

	// Make the socket non-blocking
	flags = fcntl( sockfd, F_GETFL );
	flags |= O_NONBLOCK;
	fcntl( sockfd, F_SETFL, flags );

	bzero((char *) &serv_addr, sizeof(serv_addr));

	portno = pServer->u16Port;

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);

	if( bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0 )
	{
		fprintf(stderr, "SOCKET Server: bind Error: %s\n", strerror(errno));
		pthread_exit(0);
	}

	// We're done initializing and binding the network server stuff. Don't do it again!
	pServer->bStarted = true;

	if( listen( sockfd, SOCKET_MAX_CLIENT_CONNECTIONS ) < 0 )
	{
		fprintf(stderr, "SOCKET Server: listen Error: %s\n", strerror(errno));
		pthread_exit(0);
	}

	clilen = sizeof(cli_addr);

	printf("SOCKET Server: Waiting for client connections on port %i ...\n", portno);

	while( false == pServer->bStopServer )
	{
		// Note: Let the server accept more connections than necessary so they can be rejected if too many clients have connected.
		// "accept" call will NOT block here while accepting client connections so that proper system shutdowns can occur.
		newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, &clilen );

		if( newsockfd < 0 )
		{
			switch( errno )
			{
			case EWOULDBLOCK:
				// No socket connections were available on the NON blocking socket. Just proceed
				// to do the while loop and check if this thread should be shut down.
				// i.e. check status of pServer->bStopServer flag.
				break;

			default:
				fprintf(stderr, "SOCKET Server: accept error: %s\n", strerror(errno));
				break;
			}

			// Delay a bit so we don't chew up CPU time
			usleep(250 * 1000);
		}
		else
		{
			// Make a new client but first see if any are available in our alloted number of clients
			SOCKET_CLIENT_INFO *ptFreeClientInfo = NULL;

			// Determine which client info is free to use
			for( i=0; i<SOCKET_MAX_CLIENT_CONNECTIONS; i++ )
			{
				if( false == gtClientInfo[i].bConnected )
				{
					// Found a free one
					ptFreeClientInfo = &gtClientInfo[i];
					break;	// Yeah, I know ...
				}
			}

			if( ptFreeClientInfo != NULL )
			{
				// Start a new Client Thread
				pthread_t client_thread;

				// Initialize client connection
				ptFreeClientInfo->socket_fd = newsockfd;

				if( pthread_create( &client_thread, NULL, ClientThread, ptFreeClientInfo ) < 0 )
				{
					printf("SOCKET Server: Error: Starting new Client thread failed\n");
				}
				else
				{
					pServer->u16NumClientsConnected++;

					// Print some info about the new client
					char client_ip[INET_ADDRSTRLEN];
					inet_ntop(AF_INET, &(cli_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
					printf("SOCKET Server: Connected Client: %s\n", client_ip );
					printf("SOCKET Server: Number of Clients connected: %i\n", pServer->u16NumClientsConnected );
				}
			}
			else
			{
				printf("SOCKET Server: Error: Too many Client connections. Max is %i\n", SOCKET_MAX_CLIENT_CONNECTIONS);
				shutdown( newsockfd, SHUT_RDWR );
				close( newsockfd );
			}
		}
	}

	printf("SOCKET: Server disconnecting clients and shutting down\n");
	DisconnectClients();

	pServer->u16NumClientsConnected = 0;
	pServer->bStarted = false;

	pthread_exit(0);
}

//******************************************************************************
//
//	PollTransmitData
//
//	This function looks for data ready to be sent to the connected client.
//  Typically, this function will pull data from a buffer that an application
//  fills perhaps from a queue of some sort.
//  If data is ready, it will send it out
//
//	Parameters:
//		ptClientInfo - pointer to a SOCKET_CLIENT_INFO structure
//
//	Returns:
//		true: Data was sent
//		false: Data was not sent
//
//******************************************************************************
static bool PollTransmitData( SOCKET_CLIENT_INFO *ptClientInfo )
{
	static char acData[SOCKET_MAX_BUFFER_LENGTH];
	int nSize, i;

	nSize = ptClientInfo->TxQueue.size();

	// Only deque SOCKET_MAX_BUFFER_LENGTH bytes at a time
	nSize = (nSize > SOCKET_MAX_BUFFER_LENGTH) ? SOCKET_MAX_BUFFER_LENGTH : nSize;

	if(0 < nSize )
	{
        for(i=0; i<nSize; i++)
        {
            acData[i] = ptClientInfo->TxQueue.front();
            ptClientInfo->TxQueue.pop();
        }

		write(
			ptClientInfo->socket_fd,
			acData,
			nSize
		);
	}

	return (nSize > 0) ? true : false;
}

//******************************************************************************
//
//	PollReceiveData
//
//	This function looks for data ready to be received by the connected client
//  If data is ready, it will read and store data and *somehow* inform the
//  application that data has arrived and is ready to be processed.
//
//	Parameters:
//		ptClientInfo - pointer to a SOCKET_CLIENT_INFO structure
//
//	Returns:
//		true: Client socket is still connected
//		false: Client has disconnected
//
//******************************************************************************
static bool PollReceiveData( SOCKET_CLIENT_INFO *ptClientInfo )
{
	bool bSuccess = true;
	int i;

	while (bSuccess )
	{
		static char acData[SOCKET_MAX_BUFFER_LENGTH];

		int nSize = read(
			ptClientInfo->socket_fd,
			acData,
			sizeof(acData)
		);

		// Has the client disconnected from us?
		if( nSize == 0 || errno == ETIMEDOUT )
		{
			printf("\tSOCKET Client: Disconnected or read error\n");
			bSuccess = false;
		}

		if (0 < nSize)
		{
            // store Rx'd data to the queue
            for(i=0; i<nSize; i++)
            {
                ptClientInfo->RxQueue.push(acData[i]);
            }

            // Call the application's data receieved call-back function
            ptClientInfo->callbackFn((int)ptClientInfo->RxQueue.size());
		}
		else
		{
            break;
        }
	}

	return bSuccess;
}

//******************************************************************************
//
//	ClientThread
//
//	This is the socket client thread that handles reads and writes to this connection
//
//	Parameters:
//		ptClientInfo - pointer to a SOCKET_CLIENT_INFO structure
//
//	Returns:
//		NONE
//
//******************************************************************************
static void *ClientThread( void *ptClientInfo )
{
	SOCKET_CLIENT_INFO *pClient = (SOCKET_CLIENT_INFO*)ptClientInfo;

	printf("\tSOCKET Client: Thread %i started\n", pClient->socket_fd);

	// Initialize client connection
	pClient->bConnected = true;
	pClient->bDisconnect = false;

	// Set reads and writes to be Non-Blocking!
	fcntl( pClient->socket_fd, F_SETFL, O_NONBLOCK );

	// Client's "main loop"
	while( false == pClient->bDisconnect )
	{
		PollTransmitData( pClient );

		if( !PollReceiveData( pClient ) )
		{
			// Client has disconnected so exit this thread
			pClient->bDisconnect = true;
		}
		usleep( SOCKET_POLLING_INTERVAL );
	}

	printf("\tSOCKET Client: Thread %i shutting down\n", pClient->socket_fd );

	// Shut down client connection stuff
	shutdown( pClient->socket_fd, SHUT_RDWR );
	close( pClient->socket_fd );
	pClient->socket_fd = -1;
	pClient->bConnected = false;

	gtSocketServerInfo.u16NumClientsConnected--;

	pthread_exit(0);
}

