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

#define DEBUG_HTTPSRV         0       /* Debug Http server level */

//#define DEBUG_OUTPUT(lvl,...)             DEBUG_PRINTF(__VA_ARGS__)  
#define DEBUG_OUTPUT(lvl,...)             do if ( debuglevel > lvl ) { DEBUG_PRINTF(__VA_ARGS__); } while(0)  

#define HTTPDDEBUG(...)                   DEBUG_OUTPUT(0, __VA_ARGS__)

#if DEBUG_HTTPSRV > 0   /***** Lvl 1 *****/
    #define HTTPD1DEBUG(...)              DEBUG_OUTPUT(1, __VA_ARGS__)
#else
    #define HTTPD1DEBUG(...)   
#endif
#if DEBUG_HTTPSRV > 1   /***** Lvl 2 *****/
    #define HTTPD2DEBUG(...)              DEBUG_OUTPUT(2, __VA_ARGS__)
#else
    #define HTTPD2DEBUG(...)   
#endif
#if DEBUG_HTTPSRV > 2   /***** Lvl 3 *****/
    #define HTTPD3DEBUG(...)              DEBUG_OUTPUT(3, __VA_ARGS__)
#else
    #define HTTPD3DEBUG(...)   
#endif
#if DEBUG_HTTPSRV > 3   /***** Lvl 4 *****/
    #define HTTPD4DEBUG(...)              DEBUG_OUTPUT(4, __VA_ARGS__)
#else
    #define HTTPD4DEBUG(...)   
#endif


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
        struct {
            PageDetail  DetailCB;   /* Function to fill in the page details   */
            PageDetail  SetterCB;   /* Function to handle input via GET       */
        };
    };
} OnePageT;

#define MAX_TOKEN_LEN           80          /* max length of one Get-token (params excluded) */
#define MAX_PARAM_LEN           160         /* max length of the ?param=value section        */
#define MAX_PARAM               10          /* max number of parameter/value pairs           */

typedef struct {                            /* structure to point to one param/value pair    */
    uint8_t p_ofs;                          /* offset of the param string                    */
    uint8_t v_ofs;                          /* offset of the value string                    */
} ParamValuePairT;

typedef struct {
    char        paramstr[MAX_PARAM_LEN+1];  /* String to hold the whole parameter string     */
    ParamValuePairT pv[MAX_PARAM];          /* Array to hold ptrs to PV-Pairs                */
    uint32_t         num_params;            /* Actual number of valid PV-Pairs               */
} HttpGetParamT;


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
void HtmlSetCM7         ( struct netconn *conn, void *arg); 
void HtmlSettingsLocal  ( struct netconn *conn, void *arg); 
void HtmlSetLocal       ( struct netconn *conn, void *arg); 
void HtmlShowEthIf      ( struct netconn *conn, void *arg);

/**********************************************************************************************************************************
 * Web pages file and menu structures
 * @note: Home page has to be the first entry -> see HOME_PAGE_IDX
 *********************************************************************************************************************************/
static const OnePageT WebPages[] = {
    { "STM32H745 WebServer", "Startseite",   "/",                    0, NULL,              PDstatic,  .FileName = "/home.html" },
    { "Task List",           "Tasklist",     "/tasks.html",          5, &TaskListPageHits, PDdynamic, .DetailCB = HtmlTaskList,      .SetterCB = NULL },
#if defined(DUAL_CORE)
    { "Settings CM7",        "Settings CM7", "/settings_cm7.html",   0, NULL,              PDdynamic, .DetailCB = HtmlSettingsCM7,   .SetterCB = HtmlSetCM7 },
    { "Settings CM4",        "Settings CM4", "/settings_cm4.html",   0, NULL,              PDdynamic, .DetailCB = HtmlSettingsLocal, .SetterCB = HtmlSetLocal },
#else
    { "Settings",            "Settings CM7", "/settings_cm7.html",   0, NULL,              PDdynamic, .DetailCB = HtmlSettingsLocal, .SetterCB = HtmlSetLocal },
#endif
    { "ETH IF Statistic",    "ETH Interface","/eth_if.html",         0, NULL,              PDdynamic, .DetailCB = HtmlShowEthIf,     .SetterCB = NULL },
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
        HTTPD2DEBUG("httpd open file %s\n", filename);
        netconn_write(conn, (const unsigned char*)(file.data), (size_t)file.len, NETCONN_NOCOPY);
        fs_close(&file);
    } else { 
        HTTPD2DEBUG("httpd failed to open file %s\n", filename);
        netconn_write(conn, FileErr, strlen(FileErr), NETCONN_NOCOPY);
        netconn_write(conn, filename, strlen(filename), NETCONN_COPY);
    }
}

/******************************************************************************
 * transfer WebPages[page_idx] via conn
 *****************************************************************************/
static void HtmlPage(struct netconn *conn, uint32_t page_idx, HttpGetParamT *p )
{
    static const char PageErr[] = "<br><br> No PageType implementation found<br>";

    // assert(page_idx < sizeof(WebPages)/sizeof(OnePageT));
    HtmlHeader(conn, page_idx);
    switch ( WebPages[page_idx].PageType ) {
        case PDstatic:
            HtmlStaticFile(conn, WebPages[page_idx].FileName);
            break;
        case PDdynamic:
            /* If we have parameters behind GET, first execute the Setter callback, if specfied */
            if ( p->num_params > 0 && WebPages[page_idx].SetterCB ) WebPages[page_idx].SetterCB(NULL, (void *)p );
            /* Thereafter actualize the dynamic page */
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
static void HandleGet ( struct netconn *conn, char *buf, HttpGetParamT *p )
{
    HTTPD1DEBUG("HandleGet:%s:\n", buf);
 
    /* If requested file is one of the dynamically created ones, then create it */
    for ( uint32_t i = 0; i < sizeof(WebPages)/sizeof(OnePageT); i++ ) {
        if ( CHECK(WebPages[i].HTMLPageName) ) {
            HtmlPage(conn, i, p );
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
        HtmlPage(conn, HOME_PAGE_IDX, p);
        return;
    } 
 
    /* If all failed, load Error page */
    HtmlStaticFile(conn, HTML_404); 
}

static void ParseParams(char *buf, u16_t buflen, HttpGetParamT *p )
{
    char *src;
    char *dest;
    uint32_t i = 0;
    uint32_t pcnt = 0;

    /* Default return value: No parameter/value pairs */
    p->num_params = 0;

    /* Make sure, the parameter prefix is first sign */
    if (*buf != '?' ) return;
    
    /* skip first '?' */
    buf++; buflen--;

    /* assert we have params */
    if ( buflen == 0 ) return;
    
    src  = buf;
    dest = p->paramstr;
    /* Copy the buffer from begin to first blank to "paramstr" and add terminating '\0' */
    while ( i < MAX_PARAM_LEN && i < buflen && *src != ' ' ) { 
        *(dest++) = *(src++);
        i++;
    }
    *dest='\0';

    /* Paramstr now contains parameter/value pairs in the format p1=v1&p2=v2... */
    HTTPD2DEBUG("Params:%s:\n", p->paramstr);
    
    /* Walk thru paramstr and find all parameter value pairs ) */
    src     = p->paramstr;
    i       = 0;
    pcnt    = 0;
    
    do {
        /* Store offset of parameter name */
        p->pv[pcnt].p_ofs = i;
        /* find corresponding value by by scanning for '=' */
        src++;i++;
        while ( *src && *src != '=' ) { 
            src++;i++;
        }
        /* '=' not found -> terminate parse */
        if ( !(*src) ) break;

        /* otherwise replace '=' with '\0' and store offset of value */
        *(src)='\0';
        src++;i++;

        p->pv[pcnt].v_ofs = i;
        pcnt++;

        /* Find next parameter by scanning for & */ 
        while ( *src && *src != '&' ) { 
            src++;i++;
        }

        /* No more parameter found -> terminate parse */
        if ( !(*src) ) break;

        /* Otherwise replace '&' with '\0' and continue parsing */
        *(src)='\0';
        src++;i++;
    } while (1);


    for ( i=0; i < pcnt;i++ )
        HTTPD3DEBUG("Param %d: P=%s, V=%s\n", i, p->paramstr+p->pv[i].p_ofs, p->paramstr+p->pv[i].v_ofs);

    p->num_params =  pcnt;


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
  /* token will contail the parameter behind GET up to the first blank or '?' */
  char token[MAX_TOKEN_LEN+1];
  /* params will contain the parameter string, ie. the part from first ? to next blank */
  HttpGetParamT params;
  
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
        *dest= '\0';

        /* Report when token is truncated */
        if ( i >= MAX_TOKEN_LEN ) DEBUG_PUTS("http_server: token too long, truncated");

        /* Extract and parse all parameter/value pairs of HTML GET string */
        ParseParams(src, buflen - i, &params );
        HandleGet( conn, token, &params);
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
    uint16_t remote = current->pcb.tcp->remote_port;

    /* serve connection */    
    HTTPD2DEBUG("Established connection to remote Port %d...\n",remote );

    http_server_serve(current);

    /* delete connection */
    netconn_delete(current);
    HTTPD2DEBUG("Terminated connection to remote Port %d...\n",remote );
    vTaskDelete(NULL);
    for ( ;; );
}

static void http_server_netconn_thread(void *arg)
{ 
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  
  UNUSED(arg);
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  if (conn!= NULL)
  {
    HTTPDDEBUG("Listening on Port %d...\n",PORT);
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
            sys_thread_new("HTTP Server", http_server_session, newconn, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO+1);
            taskYIELD();
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
  sys_thread_new("HTTP Listener", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
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
    #if defined(DUAL_CORE)
        MSGD_GetTasklistLine(true, prefix);
        char *ret = MSGD_WaitForTasklistLine();
        while ( ret ) {
            netconn_write(conn, ret, strlen(ret), NETCONN_COPY);
            MSGD_GetTasklistLine(false, prefix);
            ret = MSGD_WaitForTasklistLine();
        }
    #endif
}

#define ONLY_CHNG_JS    "/js/only_chgd.js"
#define SETTING_LEN 160
static char text_setting_row[] =
" <tr><td><label for=\"%s\">%s:</label>"\
"</td><td><input type=\"text\" value =\"%s\" name=\"%s\" onchange=\"add(this)\">" \
"</td></tr>\n";
#define FORMAT_TEXTLINE(result,maxlen,lbl,tag,data) snprintf(result, maxlen, text_setting_row, tag, lbl, data, tag)

static void HtmlOneTextSetting(struct netconn *conn, const char *label, const char *tag, const char *data)
{
    portCHAR line[SETTING_LEN];
    FORMAT_TEXTLINE(line, SETTING_LEN, label, tag, data);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
}
static char row_prefix[] = 
    "<tr><td><label>%s:</td><td>";
static char bool_setting_rowyn[] =
    "<input type=\"radio\" value=%s id=\"%s\" %s name=\"%s\" onchange=\"add(this)\"><label for=\"%s\">%s</label>";
static char row_postfix[] = 
    "</td></label></tr>\n";
#define FORMAT_BOOLPREFIX(result,maxlen,tag)                  snprintf(result, maxlen, row_prefix, tag)
#define FORMAT_BOOLLINE1(result,maxlen,fmt,tag,data,truetxt)  snprintf(result, maxlen, fmt, "1", "1", data==1?"checked":"", tag, "1", truetxt)
#define FORMAT_BOOLLINE0(result,maxlen,fmt,tag,data,falsetxt) snprintf(result, maxlen, fmt, "0", "0", data==0?"checked":"", tag, "0", falsetxt)
static void HtmlOneBoolSetting(struct netconn *conn, const char *label, const char *tag, uint8_t data, const char *truetxt, const char *falsetxt)
{
    portCHAR line[SETTING_LEN];
  
    FORMAT_BOOLPREFIX(line, SETTING_LEN, label);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
    FORMAT_BOOLLINE1(line, SETTING_LEN, bool_setting_rowyn, tag, data, truetxt);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
    FORMAT_BOOLLINE0(line, SETTING_LEN, bool_setting_rowyn, tag, data, falsetxt);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
    netconn_write(conn, row_postfix, strlen(row_postfix), NETCONN_NOCOPY );
}

static char decimal_setting_lbl[] =
"<tr><td><label for=\"%s\">%s(%d..%d) :</label></td>";
static char decimal_setting_val[] =
"<td><input type=\"number\" value =%d name=\"%s\" min=%d max=%d onchange=\"add(this)\"></td></tr>\n";
#define FORMAT_DECLBL(result,maxlen,lbl,tag,minval,maxval)  snprintf(result, maxlen, decimal_setting_lbl, tag, lbl, minval, maxval)
#define FORMAT_DECVAL(result,maxlen,tag,val,minval,maxval)  snprintf(result, maxlen, decimal_setting_val, val, tag, minval, maxval)

static void HtmlOneDecimalSetting(struct netconn *conn, const char *label, const char *tag, uint32_t value, uint32_t minval, uint32_t maxval)
{
    portCHAR line[SETTING_LEN];
  
    FORMAT_DECLBL(line, SETTING_LEN, label, tag, minval, maxval);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
    FORMAT_DECVAL(line, SETTING_LEN, tag, value, minval, maxval);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
}

static char hex_setting_lbl[] =
"<tr><td><label for=\"%s\">%s(%X..%X) :</label></td>";
static char hex_setting_val[] =
"<td><input pattern=\"0x[A-Fa-f0-9]{1,%d}\" value =\"%s\" name=\"%s\" onchange=\"add(this)\"></td></tr>\n";
#define FORMAT_HEXLBL(result,maxlen,lbl,tag,minval,maxval)  snprintf(result, maxlen, hex_setting_lbl, tag, lbl, minval, maxval)
#define FORMAT_HEXVAL(result,maxlen,tag,strval,hexlen,minval,maxval)  snprintf(result, maxlen, hex_setting_val, hexlen, strval, tag)

static void HtmlOneHexSetting(struct netconn *conn, const char *label, const char *tag, uint32_t value, uint32_t hexlen, uint32_t minval, uint32_t maxval)
{
    portCHAR line[SETTING_LEN];
    portCHAR strval[11];

    /* Generate the format string for printing a hex number with hexlen digits */
    snprintf(line,SETTING_LEN, "0x%%0%dx", hexlen);

    /* Generate the hex string */
    snprintf(strval, 11, line, value );
  
    FORMAT_HEXLBL(line, SETTING_LEN, label, tag, minval, maxval);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
    FORMAT_HEXVAL(line, SETTING_LEN, tag, strval, hexlen, minval, maxval);
    netconn_write(conn, line, strlen(line), NETCONN_COPY);
}

#if defined(DUAL_CORE)
    const char cm7_header[]     ="<p><b>Core CM7 Settings</b></p>";
    const char local_header[]   ="<p><b>Core CM4 Settings</b></p>";
#else
    const char local_header[]   ="<p><b>Core CM7 Settings</b></p>";
#endif

const char ethif_header[]   ="<p><b>Eth interface statistic</b></p>";
const char form_prefix[]    ="<form onSubmit=\"before_submit()\"><table>\n";
const char form_postfix[]   ="</table>\n<br><input type=\"submit\" value=\"Submit\"></form>";

#define  TAG_PREFIX         "V"
#include "eeprom.h"
void HtmlOneSetting(struct netconn *conn, MSgSettingItemT *setting )
{
    #define TAGLEN      6
    char tag[TAGLEN];

    /* Prepare the Tag. That is "V<x>", where <x> is the index of the setting element */
    snprintf(tag, TAGLEN, TAG_PREFIX"%d", setting->idx);

    /* Check for boolean value */
    switch ( setting->type ) {
    case EEType_OnOff:
        HtmlOneBoolSetting(conn, setting->help, tag, setting->val, "On", "Off");
        break;
    case EEType_YesNo:
        HtmlOneBoolSetting(conn, setting->help, tag, setting->val, "Yes", "No");
        break;
    case EEType_Uint8_Dec:
        HtmlOneDecimalSetting(conn, setting->help, tag, setting->val, setting->min, setting->max );
        break;
    case EEType_Uint8_Hex:
        HtmlOneHexSetting(conn, setting->help, tag, setting->val, 2, setting->min, setting->max );
        break;
    case EEType_Uint16_Hex:
        HtmlOneHexSetting(conn, setting->help, tag, setting->val, 4, setting->min, setting->max );
        break;
    default:
        HTTPDDEBUG("HTML rendering for setting type %d not implemented\n", setting->type);
    }
}

void HtmlSettingsTEST    ( struct netconn *conn, void *arg) 
{
    UNUSED(arg);
    netconn_write(conn, local_header, strlen(local_header), NETCONN_NOCOPY );
    netconn_write(conn, form_prefix, strlen(form_prefix), NETCONN_NOCOPY );
    HtmlStaticFile(conn, ONLY_CHNG_JS );
    HtmlOneTextSetting(conn, "Vorname", "vn", "Rainer");    
    HtmlOneBoolSetting(conn, "Name",    "nn", 1, "Yes", "No");    
    HtmlOneBoolSetting(conn, "Keks",    "xn", 0, "On", "Off"); 
    HtmlOneDecimalSetting(conn,"Decval", "dn", 24,0,255);   
    HtmlOneHexSetting(conn,"Hexval2", "dx", 0xab, 2, 0,0xff);   
    HtmlOneHexSetting(conn,"Hexval4", "dy", 0xabcd, 4, 0,0xffff);   
    HtmlOneHexSetting(conn,"Hexval8", "dz", 0xdeadbeef, 8, 0,0xffffffff);   
    netconn_write(conn, form_postfix, strlen(form_postfix), NETCONN_NOCOPY );
}

/******************************************************************************
 * extract the Tag index from the Tagstring. The Tagstring has the format
 * TAG_PREFIX<idx>. So first ensure the tag prefixx is in front, then decode
 * the index and return as number
 * -1 is returned in case of failure ( tag string or index not found )
 *****************************************************************************/
int32_t GetTagIndex( char *tagstr )
{
    size_t len = strlen(TAG_PREFIX);
    char *uuu;
    /* check for TAG_Prefix */
    if ( strncmp(TAG_PREFIX, tagstr, len ) != 0 ) return -1;

    /* Skip prefix and decode number */
    tagstr += len;
    int32_t ret = strtol(tagstr, &uuu, 10 );

    /* if uuu does not point to the end of string, there were non numeric characters */
    if (*uuu) return -1;

    return ret;
}

/******************************************************************************
 * extract the value from valstr. Valstr can be numeric ( decimal or hex )
 * alphanumeric or boolean, see EETypeT in eeprom.h
 * decoded value is returned in ret
 * function returns -1, if a conversion error occured
 *****************************************************************************/
int32_t GetTagValue ( char *valstr, uint8_t type, uint32_t *ret )
{
    char *uuu;
    /* decode all kinds of decimal values */
    switch ( type ) {
    case EEType_OnOff:
    case EEType_YesNo:
    case EEType_Uint8_Dec:
    case EEType_Uint16_Dec:
    case EEType_Uint32_Dec:
        *ret = (uint32_t)strtol(valstr, &uuu, 10 );
        /* if there are additional characters, the input is not numeric */
        if ( *uuu ) return -1;

        /* no further range checking here !*/
        break;
    case EEType_Uint8_Hex:
    case EEType_Uint16_Hex:
    case EEType_Uint32_Hex:
        /* Check for 0x at the beginning, if so skip these */
        if ( *valstr == '0' && ( *(valstr+1)=='x' || *(valstr+1)=='X' ) ) valstr += 2;
        /* Decode hex */
        *ret = (uint32_t)strtol(valstr, &uuu, 16 );
        /* if there are additional characters, the input is not numeric */
        if ( *uuu ) return -1;
        break;
    default:
        HTTPDDEBUG("HTML extract value for type %d not implemented\n", type);
    }

    return 0;
}

#if defined(DUAL_CORE)
    void HtmlSettingsCM7    ( struct netconn *conn, void *arg) 
    {
        MSgSettingItemT *ret;

        UNUSED(arg);
    
        /* remote settings */
        netconn_write(conn, cm7_header, strlen(cm7_header), NETCONN_NOCOPY );
        netconn_write(conn, form_prefix, strlen(form_prefix), NETCONN_NOCOPY );
        HtmlStaticFile(conn, ONLY_CHNG_JS );

        MSGD_GetSettingsLine(true);
        ret = MSGD_WaitForGetSettingsLine();
        while ( ret->bIsValid ) {
            HtmlOneSetting( conn, ret );
            MSGD_GetSettingsLine(false);
            ret = MSGD_WaitForGetSettingsLine();
        }
        netconn_write(conn, form_postfix, strlen(form_postfix), NETCONN_NOCOPY );
    }

    void HtmlSetCM7         ( struct netconn *conn, void *arg)
    {
        HttpGetParamT *p = (HttpGetParamT *)arg;
        int32_t idx;
        uint32_t val;
        uint32_t num_settings;
        MSgSettingItemT *ret;

        UNUSED(conn);

        /* Read first element of CM7 settings to get the number of setting items */
        MSGD_GetSettingsLine(true);
        ret = MSGD_WaitForGetSettingsLine();
        num_settings = ret->max_idx;

        /* Iterate thru all PV-pairs */
        for ( uint32_t i = 0; i < p->num_params; i++ ) {
            idx = GetTagIndex(p->paramstr+p->pv[i].p_ofs);
            if ( (uint32_t)idx > num_settings ) {
                HTTPDDEBUG("SetCM7: Index %d out of bounds\n", idx  );
                continue;
            }
            if ( idx >= 0 && GetTagValue(p->paramstr+p->pv[i].v_ofs, eelimits[idx].type, &val) >= 0 ) {
                MSGD_SetSettingsLine((uint8_t) idx, (uint8_t) val);
                if ( !MSGD_WaitForSetSettingsLine() )
                    HTTPDDEBUG("SetCM7: Failede to set config[%d] to %d\n", idx,val  );
            }            
        }
    }
#endif

static void GetLocalSettings(uint32_t idx, MSgSettingItemT *ret)
{
    ret->bIsValid   = true;
    ret->idx        = idx;
    ret->help       = eelimits[idx].help;
    ret->max        = eelimits[idx].max;
    ret->min        = eelimits[idx].min;
    ret->type       = eelimits[idx].type;
    ret->val        = Config_GetVal(idx);
}

void HtmlSettingsLocal    ( struct netconn *conn, void *arg) 
{
    MSgSettingItemT ret;
    UNUSED(arg);

    /* remote settings */
    netconn_write(conn, local_header, strlen(local_header), NETCONN_NOCOPY );
    netconn_write(conn, form_prefix, strlen(form_prefix), NETCONN_NOCOPY );
    HtmlStaticFile(conn, ONLY_CHNG_JS );
    
    uint32_t num = Config_GetCnt();
    for ( uint32_t i = 0; i < num; i++ ) {
        GetLocalSettings(i, &ret);
        HtmlOneSetting( conn, &ret );
    }
    netconn_write(conn, form_postfix, strlen(form_postfix), NETCONN_NOCOPY );
}

void HtmlSetLocal         ( struct netconn *conn, void *arg)
{
    HttpGetParamT *p = (HttpGetParamT *)arg;
    int32_t idx;
    uint32_t val;

    UNUSED(conn);

    for ( uint32_t i = 0; i < p->num_params; i++ ) {
        idx = GetTagIndex(p->paramstr+p->pv[i].p_ofs);
        if ( (uint32_t)idx > Config_GetCnt() ) {
            HTTPDDEBUG("SetCM4: Index %d out of bounds\n", idx  );
            continue;
        }
        if ( idx >= 0 && GetTagValue(p->paramstr+p->pv[i].v_ofs, eelimits[idx].type, &val) >= 0 ) {
            if ( !Config_SetVal((uint8_t) idx, (uint8_t) val) )
                HTTPDDEBUG("SetCM4: Failed to set config[%d] to %d\n", idx,val  );
        }            
    }
}

/* Ethernet driver must implement these two statistic functions */
uint32_t ETHSTAT_GetLineCount ( void );
char*    ETHSTAT_GetLine      ( char *retbuf, size_t buflen, uint32_t idx );

/******************************************************************************
 * HTML function to dump ETH interface statistics
 *****************************************************************************/
void HtmlShowEthIf ( struct netconn *conn, void *arg) 
{
    UNUSED(arg);

    /* remote settings */
    netconn_write(conn, ethif_header, strlen(ethif_header), NETCONN_NOCOPY );

    portCHAR line[MAXLEN];
    uint32_t linecount;
    char *ret;

    /* Task List */
    netconn_write(conn, "<pre>", 5, NETCONN_COPY);

    linecount = ETHSTAT_GetLineCount();
    for ( uint32_t i=0; i < linecount; i++ ) {
        ret = ETHSTAT_GetLine(line, MAXLEN, i );
        if ( ret ) netconn_write(conn, line, strlen(line), NETCONN_COPY);
    }
    
}


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
