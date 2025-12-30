/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  tcpClientRAW.h
  Author:     ControllersTech.com
  Updated:    29-Jul-2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


#ifndef INC_TCPCLIENTRAW_H_
#define INC_TCPCLIENTRAW_H_



/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  	   tcpClientRAW.c
  Modified By:     ControllersTech.com
  Updated:    	   29-Jul-2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/


/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable  application.
 *
 **/

 /* This file was modified by ST */

#include "lwip/tcp.h"
#include <string.h>
#include "ControlerConfig.h"

#define ADDR_IDX	0
#define LEN_IDX		1


#define END_OF_TRANSMISSION_CODE	0xFF

#define SET_CODE	0x00
#define GET_CODE	0x01


#define TARGET_SPEED_CODE	0x70
#define TARGET_POS_CODE		0x71
#define ACCELERATION_CODE	0x72
#define DECELERATION_CODE	0x73
#define MAX_TORQUE_CODE		0x74



/*  protocol states */
enum tcp_client_states
{
  ES_NONE = 0,
  ES_CONNECTED,
  ES_RECEIVING,
  ES_CLOSING
};

/* structure for maintaining connection infos to be passed as argument
   to LwIP callbacks*/
struct tcp_client_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};



int counter = 0;
struct tcp_pcb *pcbTx = 0;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	char buf[100];
//
//	/* Prepare the first message to send to the server */
//	int len = sprintf (buf, "Sending TCPclient Message %d\n", counter);
//
//	if (counter !=0)
//	{
//		/* allocate pbuf */
//		esTx->p = pbuf_alloc(PBUF_TRANSPORT, len , PBUF_POOL);
//
//
//		/* copy data to pbuf */
//		pbuf_take(esTx->p, (char*)buf, len);
//
//		tcp_client_send(pcbTx, esTx);
//
//		pbuf_free(esTx->p);
//	}
//
//}

void ClientCloseConnection(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{

  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);

  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }

  /* close tcp connection */
  tcp_close(tpcb);
}



static void ClientSendString(struct tcp_pcb *tpcb, const char *msg)
{
    u16_t len = strlen(msg);
    err_t err;

    while(len > 0) {

        u16_t snd = tcp_sndbuf(tpcb);
        u16_t chunk = (len < snd) ? len : snd;

        err = tcp_write(tpcb, msg, chunk, TCP_WRITE_FLAG_COPY);
        if(err != ERR_OK)
            break;

        msg += chunk;
        len -= chunk;
    }

    tcp_output(tpcb);
}




/*
 * |------------|-----------|-------------------------------------------------------|-----------------------------------------------|-------|
 * |   ADDRES	|	 LEN 	|	 CODE	|  SET/GET	|  DATA n 	|   DATA n-1 ...	|	 CODE	|  SET/GET	|  DATA 1 	|   DATA 0	|
 * |------------|-----------|-------------------------------------------------------|-----------------------------------------------|-------|
 *
 *
 */

void ClientHandleRecievedData (struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
	/* get the Remote IP */
	ip4_addr_t inIP = tpcb->remote_ip;
	uint16_t inPort = tpcb->remote_port;

	/* Extract the IP */
	char *remIP = ipaddr_ntoa(&inIP);

	uint32_t byteCount = 0;

	/*
	//1. létrehoz egy q pbuf másolatot ami kezdéskor az es->p -re mutat
	//2. végig megy a teljes láncon
	//3. egészen amíg el nem éri a végét (NULL)
	for(struct pbuf *q = es->p; q != NULL; q = q->next)
	{
		//4. Át castolja a void payload-ot byte típússá
		uint8_t *buff = (uint8_t *)q->payload;

		//5. végig iterál a pbuf payload byteokon
		for(uint32_t i = 0; i < q->len; i++)
		{
			byteCount += 1;

			switch (buff[i])
			{
				case 0x00:
					ClientSendString(tpcb, "NULL");
					break;

				case 0x01:
					ClientSendString(tpcb, "0x01");
					break;
			}
		}

	}
	*/

	//Dinamikusan lefoglal egy kellően nagy tömböt
	uint8_t *buf = malloc(es->p->tot_len);
	if(buf == NULL)
	{
		return;
	}

	//A TCP-ből érkező adatokat átmásolja a buf tömbbe
	pbuf_copy_partial(es->p, buf, es->p->tot_len, 0);

	//Ellenőrzi, hogy az üzenet neki szól
	if(buf[ADDR_IDX] != 0x32)
	{
		free(buf);
		return;
	}


	uint8_t *p = NULL;
	uint8_t s = 0;

	//Végig iterál a byteokon
	for (uint16_t i = 2; i < es->p->tot_len;)
	{
		uint8_t Code = buf[i++];
		uint16_t SetGet = buf[i++];


		//A kód függvényében beírja a megfelelő adatot
		switch(Code)
		{
		case TARGET_SPEED_CODE:
			//ControlConfig.TargetSpeed = ((uint16_t)(*(buf + (i+1))) << 8) | ((uint16_t)(*(buf + (i+2))));
			p = (uint8_t *)&ControlConfig.TargetSpeed;
			s = sizeof(ControlConfig.TargetSpeed);
			break;

		case TARGET_POS_CODE:
			//ControlConfig.TargetPos = ((uint16_t)(*(buf + (i+1))) << 8) | ((uint16_t)(*(buf + (i+2))));
			p = (uint8_t *)&ControlConfig.TargetPos;
			s = sizeof(ControlConfig.TargetPos);
			break;
		}

		if(i + s > es->p->tot_len)
		{
			return;
		}




		switch(SetGet)
		{
		case SET_CODE:
			memcpy(p, &buf[i], s);
			break;


		case GET_CODE:
			uint32_t val = 0;

			memcpy(&val, p, s);

			char out[64];
			sprintf(out, "Val=%lu\n", (unsigned long)val);
			ClientSendString(tpcb, out);
			break;
		}

		i += s;
	}
	free(buf);


	char buff[100];
	sprintf (buff, "\n Target speed: %d\n Target pos: %d\n  ", ControlConfig.TargetSpeed, ControlConfig.TargetPos);

	ClientSendString(tpcb, buff);
//	esTx->state = es->state;
//	esTx->pcb = es->pcb;
//	esTx->p = es->p;

	//pcbTx = tpcb;

	counter++;

}

static err_t callbackDataReciev(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_client_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);

  es = (struct tcp_client_struct *)arg;

  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       /* we're done sending, close connection */
       ClientCloseConnection(tpcb, es);
    }
    else
    {
      /* we're not done yet */
//      /* acknowledge received packet */
//      tcp_sent(tpcb, callbackDataSent);

      /* send remaining data*/
//      tcp_client_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }
  /* else : a non empty frame was received from server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_CONNECTED)
  {
   /* store reference to incoming pbuf (chain) */
    es->p = p;

    // tcp_sent has already been initialized in the beginning.
//    /* initialize LwIP tcp_sent callback function */
//    tcp_sent(tpcb, callbackDataSent);

    /* Acknowledge the received data */
    tcp_recved(tpcb, p->tot_len);

    /* handle the received data */
    ClientHandleRecievedData(tpcb, es);

    pbuf_free(p);

    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unknown es->state, trash data  */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}


static err_t ClientPoll(void *arg, struct tcp_pcb *tpcb) {
	struct tcp_client_struct *es;

	es = (struct tcp_client_struct*) arg;
	if (es == NULL) {
		/* nothing to be done */
		tcp_abort(tpcb);
		return ERR_ABRT;
	}

	if (es->p == NULL && es->state == ES_CLOSING)
	{
		ClientCloseConnection(tpcb, es);
	}

	return ERR_OK;
}



/** This callback is called, when the server acknowledges the data sent by the client
 * If there is no more data left to sent, we will simply close the connection
  */
static err_t callbackDataSent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
	struct tcp_client_struct *es;

	LWIP_UNUSED_ARG(len);

	es = (struct tcp_client_struct*) arg;
	es->retries = 0;

	if (es->p == NULL && es->state == ES_CLOSING)
	{
		ClientCloseConnection(tpcb, es);
	}

	return ERR_OK;
}


/** A function to send the data to the server
  */
static void ClientSendData(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{


  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

  while ((wr_err == ERR_OK) &&
         (es->p != NULL) &&
         (es->p->len <= tcp_sndbuf(tpcb)))
  {

    /* get pointer on pbuf from es structure */
    ptr = es->p;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);

    if (wr_err == ERR_OK)
    {
      u16_t plen;
      u8_t freed;

      plen = ptr->len;

      /* continue with next pbuf in chain (if any) */
      es->p = ptr->next;

      if(es->p != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p);
      }

     /* chop first pbuf from chain */
      do
      {
        /* try hard to free pbuf */
        freed = pbuf_free(ptr);
      }
      while(freed == 0);
     /* we can read more data now */
//     tcp_recved(tpcb, plen);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}








/** This callback is called, when the client is connected to the server
 * Here we will initialise few other callbacks
 * and in the end, call the client handle function
 */
static err_t callbackClientConnected(void *arg, struct tcp_pcb *newpcb, err_t err) {

	struct tcp_client_struct *es;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	/* allocate structure es to maintain tcp connection information */
	es = (struct tcp_client_struct*) mem_malloc(sizeof(struct tcp_client_struct));

	if (es == NULL)
	{
		/*  close tcp connection */
		ClientCloseConnection(newpcb, es);
		/* return memory error */
		return ERR_MEM;
	}

	es->state = ES_CONNECTED;
	es->pcb = newpcb;
	es->retries = 0;
	es->p = NULL;

	/* pass newly allocated es structure as argument to newpcb */
	tcp_arg(newpcb, es);

	/* initialize lwip tcp_recv callback function for newpcb  */
	tcp_recv(newpcb, callbackDataReciev);

	/* initialize lwip tcp_poll callback function for newpcb */
	tcp_poll(newpcb, ClientPoll, 0);

	/* initialize LwIP tcp_sent callback function */
	tcp_sent(newpcb, callbackDataSent);

	/* handle the TCP data */
	ClientHandleRecievedData(newpcb, es);

	return ERR_OK;

}


/* IMPLEMENTATION FOR TCP CLIENT

1. Create TCP block.
2. connect to the server
3. start communicating
*/

void initTcpClient(uint8_t IP3, uint8_t IP2, uint8_t IP1, uint8_t IP0, uint16_t port)
{
	/* 1. create new tcp pcb */
	struct tcp_pcb *tpcb;

	tpcb = tcp_new();

	/* 2. Connect to the server */
	ip_addr_t destIPADDR;
	IP_ADDR4(&destIPADDR, IP3, IP2, IP1, IP0);
	tcp_connect(tpcb, &destIPADDR, port, callbackClientConnected);
}





#endif /* INC_TCPCLIENTRAW_H_ */
