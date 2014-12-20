// SocketServer.h
//
// Sets up, listens and accepts socket client connections

#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include "../includes.h"

#define SOCKET_MAX_BUFFER_LENGTH		512

// Max number of client connections
#define SOCKET_MAX_CLIENT_CONNECTIONS	1

//------------------------------------------------------------------------------
void Server_Init( void (*read_callback)(int) );
bool Server_Start( U16 port );
void Server_Stop( void );
U16 Server_GetRxData( int client, U8 *u8RxBuffer );
U16 Server_SendTxData( int client, U8 *u8TxBuffer, U16 bytes );

#endif
