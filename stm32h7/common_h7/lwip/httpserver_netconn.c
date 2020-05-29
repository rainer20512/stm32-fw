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
#include "debug_helper.h"


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
    const char  *HTMLPageName;      /* Page name as requested by browser */
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
#define HOME_PAGE_IDX   0           /* Index of home page in page list        */
#define HTML_404        "/404.html" /* static page for 404                    */

/* Private variables ---------------------------------------------------------*/
u32_t TaskListPageHits = 0;

/* Forward declarations ------------------------------------------------------*/
void HtmlHomePage       ( struct netconn *conn, void *arg); 
void HtmlTaskList       ( struct netconn *conn, void *arg); 
void HtmlSettingsCM7    ( struct netconn *conn, void *arg); 
void HtmlSettingsCM4    ( struct netconn *conn, void *arg); 


/**********************************************************************************************************************************
/* Web pages file and menu structures
 * @note: Home page has to be the first entry -> see HOME_PAGE_IDX
 *********************************************************************************************************************************/
static const OnePageT WebPages[] = {
    { "STM32H745 WebServer", "Startseite",   "/",                   0, NULL,              PDstatic,  .FileName = "/home.html" },
    { "Task List",           "Tasklist",     "tasks.html",          5, &TaskListPageHits, PDdynamic, .DetailCB = HtmlTaskList },
    { "Settings CM7",        "Settings CM7", "settings_cm7.html",   0, NULL,              PDdynamic, .DetailCB = HtmlSettingsCM7 },
    { "Settings CM4",        "Settings CM4", "settings_cm4.html",   0, NULL,              PDdynamic, .DetailCB = HtmlSettingsCM4 },
};

/* Private functions ---------------------------------------------------------*/

/* All the following static vars are used to generate the dynamic page header */
static cc HEADER[] =                                                                             \
"<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN\" \"http://www.w3.org/TR/html4/strict.dtd\">" \
"<meta http-equiv=\"Content-Type\" content=\"text/html;charset=windows-1252\">"
;
static cc TITLE_s[] =
"<html><head><title>%s</title>"                                                      
;

static cc REFRESH_d[] =                                                                          \
"<meta http-equiv=\"refresh\" content=\"%d\">"
;

static cc HEADER2[] =                                                                            \
"<style =\"font-weight:normal;font-family:Verdana\"></style>"                                    \
"</head><body>"                                                                                  \
"<table style=\"width:961px\"><tr style=\"height:100px\">"                                       \
"<td><a href=\"http://megaexperte.de\"><img style=\"width: 100px; height: 100px;\""          \
" alt=\"Idiot\" src=\"im/idiot.png\"> </a> </td>"                                                \
"<td style=\"color:rgb(51, 51, 255); font-family:Verdana; font-weight:bold; font-style:italic; font-size: 30px\">"
;

/* Header Text here */

static cc HEADER3[] =                                                                            \
"</td></tr></table>"                                                                             \
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


/******************************************************************************
 * transfer a static header, which is common to all pages via "conn"
 * This common header is outlined in the project file 
 * .\tools\makefsdata\dynamic\page_header.html, but generated dynamically at
 * runtime.
 * it consists of a headline, consisting of an image and a headline text,
 * a horizontal menu bar, consisting of all entries of "WebPages", except the
 * current page, an horizontal bar and and optional hit counter and and optional
 * refresh period
 *****************************************************************************/
static void HtmlHeader(struct netconn *conn, uint32_t page_idx )
{
    /* buffer to generate an dynamic line */
    char dyn[LINE_LEN];
    const OnePageT *actpage = &WebPages[page_idx];

    /* Static Header part 1*/
    netconn_write(conn, HEADER, strlen((char*)HEADER), NETCONN_NOCOPY);

    /* title */
    snprintf(dyn, LINE_LEN, TITLE_s, actpage->Title);
    netconn_write(conn, dyn, strlen(dyn), NETCONN_COPY);
    
    /* Refresh rate, if desired */
    uint32_t r = actpage->RefreshRate;
    if ( r == 0 ) r = 9999;
    snprintf(dyn, LINE_LEN, REFRESH_d, r);
    netconn_write(conn, dyn, strlen(dyn), NETCONN_COPY);

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
            snprintf(dyn, LINE_LEN, COL_s_s, WebPages[i].HTMLPageName, WebPages[i].MenuTitle);
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

/******************************************************************************
 * transfer the static file "filename" from the pseudo filesystem via "conn"
 * an error mesg is displayed, if not found
 *****************************************************************************/
static void HtmlStaticFile(struct netconn *conn, const char *filename )
{
    static const char FileErr[] = "File not found: ";
    struct fs_file file;

    
    if ( fs_open(&file, filename) == ERR_OK ) {
        DEBUG_PRINTF("httpd open file %s\n", filename);
        netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
        fs_close(&file);
    } else { 
        DEBUG_PRINTF("httpd failed to open file %s\n", filename);
        netconn_write(conn, FileErr, strlen(FileErr), NETCONN_NOCOPY);
        netconn_write(conn, filename, strlen(filename), NETCONN_COPY);
    }
}

/******************************************************************************
 * transfer WebPages[page_idx] via conn
 *****************************************************************************/
static void HtmlPage(struct netconn *conn, uint32_t page_idx )
{
    static const char PageErr[] = "<br><br> No PageType implementation found<br>";

    // assert(page_idx < sizeof(WebPages)/sizeof(OnePageT));
    const OnePageT *actpage = &WebPages[page_idx];
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

/******************************************************************************
 * check, whether "item" is contained as static file in fs_data
 * @param item   - NULL terminated static filename
 * @returns
 *    pdTRUE   - if "item" is a static filename
 *    pdFALSE  - otherwise
 *****************************************************************************/
static int32_t IsStaticFile ( const char *item )
{
    struct fs_file file;
    return ( fs_open(&file, item ) == ERR_OK ? fs_close(&file), pdTRUE : pdFALSE );
}

#define CHECK(file)     ( strcmp(file, buf) == 0 )

/******************************************************************************
 * Handle an HTML GET 
 * @param conn   - actual netconn
 * @param buf    - string BEHIND "GET ", null-terminated 
 * @param buflen - length of that strinf
 *****************************************************************************/
static void HandleGet ( struct netconn *conn, char *buf)
{
    DEBUG_PRINTF("HandleGet:%s:\n", buf);

    if ( CHECK("/tasks.html") ) {
       /* Load dynamic page */
       HtmlPage(conn, 1);
       // DynWebPage(conn);
       return;
    }
    else if ( CHECK("/settings_cm7.html") ) {
       /* Load dynamic page */
       HtmlPage(conn, 2);
       // DynWebPage(conn);
       return;
    }
    
    /* If requested file is one of the dynamically created ones, then create it */
    for ( uint32_t i = 0; i < sizeof(WebPages)/sizeof(OnePageT); i++ ) {
        if ( CHECK(WebPages[i].HTMLPageName) ) {
            HtmlPage(conn, i );
            return;
        }
    }

    /* If requested file is that of a static file from fs_data, then send it */
    if ( IsStaticFile(buf) ) { 
        HtmlStaticFile(conn, buf); 
        return; 
    }

    /* finally, check for index.html */
    if ( CHECK("/index.html") ) {
        HtmlPage(conn, HOME_PAGE_IDX);
        return;
    } 
 
    /* If all failed, load Error page */
    HtmlStaticFile(conn, HTML_404); 
}
#define MAX_TOKEN_LEN           80          /* max length of one Get-token (params excluded) */
#define MAX_PARAM_LEN           160         /* max length of the ?param=value section        */
#define MAX_PARAM               10          /* max number of parameter/value pairs           */

typedef struct {                            /* structure to point to one param/value pair    */
    uint8_t p_ofs;                          /* offset of the param string                    */
    uint8_t v_ofs;                          /* offset of the value string                    */
} ParamValuePairT;


static uint8_t ParseParams(char *buf, u16_t buflen, char* paramstr, ParamValuePairT *pv )
{
    char *src;
    DEBUG_PUTS("Params:");
    src=buf;
    for(uint32_t i=0; i < buflen; i++ )
        DEBUG_PUTC(*(src++));
    DEBUG_PUTC('\n');
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
  char *dest, *src;
  u16_t buflen;
  uint32_t i;
  uint32_t num_params;
  char token[MAX_TOKEN_LEN+1];
  char params[MAX_PARAM_LEN];
  ParamValuePairT pv[MAX_PARAM];
  
  /* Read the data from the port, blocking if nothing yet there. 
   We assume the request (the part we care about) is in one netbuf */
  recv_err = netconn_recv(conn, &inbuf);
  
  if (recv_err == ERR_OK)
  {
    if (netconn_err(conn) == ERR_OK) 
    {
      netbuf_data(inbuf, (void**)&buf, &buflen);

      /* Handle GET command */
      if ( (buflen >=4) && (strncmp(buf, "GET ", 4) == 0)) {
        /* extract the first token behind "GET " */
        src = buf+4; dest = token;
        buflen -= 4; i = 0;
        while ( i < MAX_TOKEN_LEN && i < buflen && *src != ' ' && *src != '?' ) { 
            *(dest++) = *(src++);
            i++;
        }
        if ( *src == '?' ) 
            num_params = ParseParams(src+1, buflen - i - 1, params, pv );
        else
            num_params = 0;

        *dest= '\0';
        /* Report when token is truncated */
        if ( i >= MAX_TOKEN_LEN ) DEBUG_PUTS("http_server: token too long, truncated");
        HandleGet( conn, token);
      }
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

#define ONLY_CHNG_JS    "/js/only_chgd.js"
static char setting_row[] =
" <tr><td><label for=\"%s\">%s:</label>"\
"</td><td><input type=\"text\" value =\"%s\" name=\"%s\" onchange=\"add(this)\">" \
"</td></tr>";
#define FORMAT_LINE(result,maxlen,lbl,tag,data) snprintf(result, maxlen, setting_row, tag, lbl, data, lbl)
#define SETTING_LEN 160

static void HtmlOneSetting(struct netconn *conn, const char *label, const char *fortag, const char *data)
{
    portCHAR line[SETTING_LEN];
    FORMAT_LINE(line, SETTING_LEN, label, fortag, data);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
}


const char form_prefix[]="<form onSubmit=\"before_submit()\"><table>";
const char form_postfix[]="</table><br><input type=\"submit\" value=\"Submit\"></form>";


void HtmlSettingsCM7    ( struct netconn *conn, void *arg) 
{
    netconn_write(conn, form_prefix, strlen(form_prefix), NETCONN_NOCOPY );
    HtmlStaticFile(conn, ONLY_CHNG_JS );
    HtmlOneSetting(conn, "Vorname", "vn", "Rainer");    
    HtmlOneSetting(conn, "Name",    "nn", "Booh");    
    netconn_write(conn, form_postfix, strlen(form_postfix), NETCONN_NOCOPY );
}

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
