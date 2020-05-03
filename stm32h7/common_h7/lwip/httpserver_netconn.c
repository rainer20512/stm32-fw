/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/httpser-netconn.c 
  * @author  MCD Application Team
  * @brief   Basic http server implementation using LwIP netconn API  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/apps/fs.h"
#include "string.h"
#include "httpserver_netconn.h"
#include "cmsis_os.h"

#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Function to call to complete the web page */
typedef void (*PageDetail) ( struct netconn *conn, void *arg);
typedef const char cc;
/* Enum to specify the type of page details ( everything below the header ) */
typedef enum {
    PDstatic    = 0,                /* static html page to be read from file */
    PDdynamic   = 1,                /* dynamically generated page            */
} PageDetailE;

/* Struct to define one web page */
typedef struct {
    const char  *Title;             /* Page title to be displayed on page */
    const char  *MenuTitle;         /* Text to be displayed as menu item */
    const char  *PageName;          /* Page name as sent to browser */
    uint32_t    RefreshRate;        /* page refresh rate, if desired */
    uint32_t    *HitCounter;        /* Hit counter for this page */
    PageDetailE PageType;           /* Type of page */
    union {                             
        const char *FileName;       /* Static file to display                 */
        PageDetail  DetailCB;       /* Function to fill in the page details   */
    };
} OnePageT;


/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO    ( osPriorityAboveNormal )

/* Private macro -------------------------------------------------------------*/
#define LINE_LEN        120

/* Private variables ---------------------------------------------------------*/
u32_t TaskListPageHits = 0;

/* Forward declarations ------------------------------------------------------*/
void HtmlHomePage       ( struct netconn *conn, void *arg); 
void HtmlTaskList       ( struct netconn *conn, void *arg); 
void HtmlSettingsCM7    ( struct netconn *conn, void *arg); 
void HtmlSettingsCM4    ( struct netconn *conn, void *arg); 

/* Web pages file and menu structures */
static OnePageT WebPages[] = {
    { "STM32H745 WebServer", "Home",         "/",                 0, NULL,              PDdynamic, .DetailCB = HtmlHomePage },
    { "Task List",           "Tasklist",     "tasklist.html",     5, &TaskListPageHits, PDdynamic, .DetailCB = HtmlTaskList },
    { "Settings CM7",        "Settings CM7", "settings_cm7.html", 0, NULL,              PDdynamic, .DetailCB = HtmlSettingsCM7 },
    { "Settings CM4",        "Settings CM4", "settings_cm4.html", 0, NULL,              PDdynamic, .DetailCB = HtmlSettingsCM4 },
};

/* Private functions ---------------------------------------------------------*/

/* All the following static vars are used to generate the dynamic page header */
static cc HEADER[] =                                                                             \
"<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN\" \"http://www.w3.org/TR/html4/strict.dtd\">" \
"<html><head><title>STM32H7xxTASKS</title>"                                                      \
"<meta http-equiv=\"Content-Type\" content=\"text/html;charset=windows-1252\">"
;
static cc REFRESH_d[] =                                                                          \
"<meta http-equiv=\"refresh\" content=\"%d\">"
;

static cc HEADER2[] =                                                                            \
"<meta content=\"MSHTML 6.00.2800.1561\" name=\"GENERATOR\">"                                    \
"<style =\"font-weight:normal;font-family:Verdana\"></style>"                                    \
"</head><body><h4 style=\"color:rgb(51, 51, 255)\"><small style=\"font-family:Verdana\">"        \
"<small><big><big><big style=\"font-weight:bold\"><big><strong><em><span style=\"font-style:italic\">"
;
/* Header Text here */
static cc HEADER3[] =                                                                            \
"</span></em></strong></big></big></big></big></small></small></h4>"                             \
"<hr style=\"width:100%;height:2px\"><span style=\"font-weight:bold\">"                          \
"</span><span style=\"font-weight:bold\">"                                                       \
"<table style=\"width:961px;height:30px\" border=\"0\" cellpadding=\"2\" cellspacing=\"10\"><tbody><tr>"
;

/* Static part of every html table column */
static cc COL_PREFIX[] =                                                                         \
"<td style=\"font-family:Verdana;font-weight:bold;font-style:italic;background-color:rgb(51, 51, 255);text-align:center\"><small>"
;

/* Dynamic part of every html table column */
static cc COL_s_s[] =                                                                            \
"<a href=\"%s\"><span style=\"color:white\">%s</span></a></small></td>"
;

/* Static part of html table potfix */
static cc TBL_POSTFIX[] =                                                                        \
"</tr></tbody></table><br></span>"                                                               \
"<span style=\"font-weight:bold\"></span><small><span style=\"font-family:Verdana\">"
;

/* Static part for Hit counter */
static cc HIT_COUNTER[] =                                                                        \
"Number of page hits:&nbsp;"
;

/* Static part for HTML body postfix, will be followed by hit counter (if desired ) */
static cc BODY_POSTFIX[] =                                                                       \
"</span></small></body></html>"
;


static void HtmlHeader(struct netconn *conn, uint32_t page_idx )
{
    /* buffer to generate an dynamic line */
    char dyn[LINE_LEN];
    OnePageT *actpage = &WebPages[page_idx];

    /* Static Header part 1*/
    netconn_write(conn, HEADER, strlen((char*)HEADER), NETCONN_NOCOPY);
    
    /* Refresh rate, if desired */
    if ( actpage->RefreshRate > 0 ) {
        snprintf(dyn, LINE_LEN, REFRESH_d, actpage->RefreshRate);
        netconn_write(conn, dyn, strlen(dyn), NETCONN_COPY);
    }

    /* Static Header part 2*/
    netconn_write(conn, HEADER2, strlen(HEADER2), NETCONN_NOCOPY);

    /* Page Title */
    netconn_write(conn, actpage->Title, strlen(actpage->Title), NETCONN_NOCOPY);

    /* Static Header part 3*/
    netconn_write(conn, HEADER3, strlen(HEADER3), NETCONN_NOCOPY);

    /* Write the page selection menu as HTML table */
    for ( uint32_t i = 0; i < sizeof(WebPages)/sizeof(OnePageT); i++ ) {
        if ( i != page_idx ) {
            /* Static column part */
            netconn_write(conn, COL_PREFIX, strlen(COL_PREFIX), NETCONN_NOCOPY);

            /* Dynamic column part */
            snprintf(dyn, LINE_LEN, COL_s_s, WebPages[i].PageName, WebPages[i].MenuTitle);
            netconn_write(conn, dyn, strlen(dyn), NETCONN_COPY);
        }
    }

    /* Static table postfix */
    netconn_write(conn, TBL_POSTFIX, strlen(TBL_POSTFIX), NETCONN_NOCOPY);

    /* Hit counter prefix, if desired */
    if ( actpage->HitCounter ) {
        netconn_write(conn, HIT_COUNTER, strlen(HIT_COUNTER), NETCONN_NOCOPY);
    }

    /* Body postfix */
    netconn_write(conn, BODY_POSTFIX, strlen(BODY_POSTFIX), NETCONN_NOCOPY);

    /* Hit counter value, if desired */
    if ( actpage->HitCounter ) {
        (*actpage->HitCounter)++;
        snprintf(dyn, LINE_LEN, "%d", *actpage->HitCounter);
        netconn_write(conn, dyn, strlen(dyn), NETCONN_COPY);
    }
}

static void HtmlStaticFile(struct netconn *conn, const char *filename )
{
    static const char FileErr[] = "File not found: ";
    struct fs_file file;

    if ( fs_open(&file, filename) == ERR_OK ) {
        netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
        fs_close(&file);
    } else { 
        netconn_write(conn, FileErr, strlen(FileErr), NETCONN_NOCOPY);
        netconn_write(conn, filename, strlen(filename), NETCONN_COPY);
    }
}

static void HtmlPage(struct netconn *conn, uint32_t page_idx )
{
    static const char PageErr[] = "<br><br> No PageType implementation found<br>";

    // assert(page_idx < sizeof(WebPages)/sizeof(OnePageT));
    OnePageT *actpage = &WebPages[page_idx];
    HtmlHeader(conn, page_idx);
    switch ( WebPages[page_idx].PageType ) {
        case PDstatic:
            HtmlStaticFile(conn, WebPages[page_idx].FileName);
            break;
        case PDdynamic:
            if ( WebPages[page_idx].DetailCB ) WebPages[page_idx].DetailCB(conn, (void *)0 );
            break;
        default:
                netconn_write(conn, PageErr, strlen(PageErr), NETCONN_NOCOPY);
    }
}



static int32_t HtmlCheckItem ( const char *item, char *buf, u16_t buflen )
{
    u16_t ilen = strlen(item);
    if (  buflen >= ilen && strncmp(buf, item, ilen)==0 ) 
        return 0;
    else
        return -1;
}

#define CHECK(file)          HtmlCheckItem(file, buf, buflen)
#define CHECKSTATIC(file)    if ( CHECK(file) ) { HtmlStaticFile(conn, file); return; }

static void HandleGet ( struct netconn *conn, char *buf, u16_t buflen )
{
    /* remove trailing blanks */    
    #if 0
    while ( buflen && *buf == ' ' )
      { buf++;buflen--; }
    #endif

    CHECKSTATIC("/STM32H7xx.html");
    CHECKSTATIC("/STM32H7xx_files/ST.gif");
    CHECKSTATIC("/STM32H7xx_files/stm32.jpg");
    CHECKSTATIC("/STM32H7xx_files/logo.jpg");
    CHECKSTATIC("/STM32H7xx_files/logo.jpg"); 
    
    if ( CHECK("/STM32H7xxTASKS.html") ) {
       /* Load dynamic page */
       HtmlPage(conn, 1);
       // DynWebPage(conn);
       return;
    }
    if ( CHECK("/") ) {
        HtmlStaticFile(conn, "/STM32H7xx.html");
    } else {
      /* Load Error page */
      HtmlStaticFile(conn, "/404.html"); 
    }
}

/**
  * @brief serve tcp connection  
  * @param conn: pointer on connection structure 
  * @retval None
  */
static void http_server_serve(struct netconn *conn) 
{
  struct netbuf *inbuf;
  err_t recv_err;
  char* buf;
  u16_t buflen;
  struct fs_file file;
  
  /* Read the data from the port, blocking if nothing yet there. 
   We assume the request (the part we care about) is in one netbuf */
  recv_err = netconn_recv(conn, &inbuf);
  
  if (recv_err == ERR_OK)
  {
    if (netconn_err(conn) == ERR_OK) 
    {
      netbuf_data(inbuf, (void**)&buf, &buflen);
      if ( (buflen >=4) && (strncmp(buf, "GET ", 4) == 0)) 
        HandleGet( conn, buf+4, buflen - 4 );

    
      /* Is this an HTTP GET command? (only check the first 5 chars, since
      there are other formats for GET, and we're keeping it very simple )*/
    }
  }
  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);
  
  /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
  netbuf_delete(inbuf);
}

#include "debug_helper.h"

/**
  * @brief  http server thread 
  * @param arg: pointer on argument(not used here) 
  * @retval None
  */
#define PORT    80
#include "lwip/tcp.h"

static void http_server_session ( void *arg )
{
    struct netconn *current = (struct netconn *)arg;
    struct tcp_pcb *tcp;
    uint16_t remote = current->pcb.tcp->remote_port;

    /* serve connection */    
    DEBUG_PRINTF("Established connection to remote Port %d...\n",remote );

    http_server_serve(current);

    /* delete connection */
    netconn_delete(current);
    DEBUG_PRINTF("Terminated connection to remote Port %d...\n",remote );
    vTaskDelete(NULL);
    for ( ;; );
}

static void http_server_netconn_thread8088(void *arg)
{ 
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  if (conn!= NULL)
  {
    DEBUG_PRINTF("Listening on Port %d...\n",PORT);
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, PORT);
    
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
  
      while(1) 
      {
        /* accept any icoming connection */
        accept_err = netconn_accept(conn, &newconn);
        if(accept_err == ERR_OK)
        {
            sys_thread_new("HTTPD", http_server_session, newconn, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO+1);
            taskYIELD();
        }
      }
    }
  }
}

static void http_server_netconn_thread80(void *arg)
{ 
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 8888);
    
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
  
      while(1) 
      {
        /* accept any icoming connection */
        accept_err = netconn_accept(conn, &newconn);
        if(accept_err == ERR_OK)
        {
          /* serve connection */
          http_server_serve(newconn);

          /* delete connection */
          netconn_delete(newconn);
        }
      }
    }
  }
}

/**
  * @brief  Initialize the HTTP server (start its thread) 
  * @param  none
  * @retval None
  */
void http_server_netconn_init()
{
//  sys_thread_new("HTTP80", http_server_netconn_thread80, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
  sys_thread_new("HTTP8088", http_server_netconn_thread8088, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
}

void HtmlHomePage       ( struct netconn *conn, void *arg) {}

/**
  * @brief  Create and send a dynamic Web Page. This page contains the list of 
  *         running tasks and the number of page hits. 
  * @param  conn pointer on connection structure 
  * @retval None
  */
#define MAXLEN      120
#include "task/minitask.h"
#include "msg_direct.h"
void HtmlTaskList(struct netconn *conn, void *arg)
{
    UNUSED(arg);

    portCHAR line[MAXLEN];
    int32_t i;
    char *ret;

    const char prefix[] = "<br>";

    /* Task List */
    netconn_write(conn, "<pre>", 5, NETCONN_COPY);
    TaskSetListStartStop(LISTMODE_HEADER, LISTMODE_BODY);

    /* local tasks */
    i = ACTIONID_INIT;
    while ( i = TaskIterateList ( i, line, 80, prefix ), i >= 0 ) {
        netconn_write(conn, line, strlen(line), NETCONN_COPY);
    }

    /* remote tasks */
    MSGD_GetTasklistLine(true, prefix);
    ret = MSGD_WaitForTasklistLine();
    while ( ret ) {
        netconn_write(conn, ret, strlen(ret), NETCONN_COPY);
        MSGD_GetTasklistLine(false, prefix);
        ret = MSGD_WaitForTasklistLine();
    }
}
void HtmlSettingsCM7    ( struct netconn *conn, void *arg) {}
void HtmlSettingsCM4    ( struct netconn *conn, void *arg) {}


#if 0
void DynWebPage(struct netconn *conn)
{
  portCHAR line[MAXLEN];

  /* Update the hit count */
  nPageHits++;

  uint32_t nroftasks = TaskGetTasks(  );

  /* Header and hit counter */
  snprintf(line, MAXLEN, "%d", (int)nPageHits);
  netconn_write(conn, PAGE_START, strlen((char*)PAGE_START), NETCONN_COPY);
  netconn_write(conn, line, strlen(line), NETCONN_COPY);

  /* Table Header */
    TaskFormatHeader(line, MAXLEN, "<pre><br>",0);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);

    for ( uint32_t i=1; TaskFormatHeader(line, MAXLEN, "<br>", i), *line; i++) {
        netconn_write(conn, line, strlen(line), NETCONN_COPY);
    }
 
    /* table content */
    for ( uint32_t i = 0; i < nroftasks; i++ ) {
        #if defined(CORE_CM7)
            TaskFormatLine(line, MAXLEN, "<br>", i, "CM7" );
        #else
            TaskFormatBody(line, MAXLEN, "<br>", i, "CM4" );
        #endif
        netconn_write(conn, line, strlen(line), NETCONN_COPY);
    }

    
  #if 0
  /* The list of tasks and their status */
  osThreadList((unsigned char *)(line + strlen(line)));
  #endif 
  /* footer */
  snprintf(line, MAXLEN, "<br>-------------------------------------------------------------------------");
  netconn_write(conn, line, strlen(line), NETCONN_COPY);
  snprintf(line, MAXLEN, "<br>A: Running, B : Blocked, R : Ready, D : Deleted, S : Suspended, I : Invalid<br>");
  netconn_write(conn, line, strlen(line), NETCONN_COPY);

  /* Send the dynamically generated page */
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
