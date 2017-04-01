/******************************************************************************
 *  Copyright (c) 2012 - 2016 Qualcomm Technologies International, Ltd.
 *  Part of CSR uEnergy SDK 2.6.0
 *  Application version 2.6.0.0
 *
 *  FILE
 *      main.c
 *
 *  DESCRIPTION
 *      Simple timers example to show Timers usage. This application also
 *      demonstrates how to produce multiple timeouts using a single timer
 *      resource by chaining timer requests.
 *
 ******************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>          /* Commonly used type definitions */
#include <ls_app_if.h>      /* Link Supervisor application interface */
#include <debug.h>          /* Simple host interface to the UART driver */
#include <timer.h>          /* Chip timer functions */
#include <panic.h>          /* Support for applications to panic */
#include <gap_app_if.h>     /* GAP application interface */
#include <mem.h>            /* Memory library */
#include <config_store.h>   /* Interface to the Configuration Store */
#include <random.h>
#include <bluetooth.h>
#include <gatt_uuid.h>      /* Common Bluetooth UUIDs and macros */
#include <pio.h>    

/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "gatt_access.h"    /* GATT-related routines */
#include "app_gatt_db.h"    /* GATT database definitions */
#include "buzzer.h"         /* Buzzer functions */
#include "nvm_access.h"     /* Non-volatile memory access */
#include "gatt_server.h"    /* Definitions used throughout the GATT server */
#include "hw_access.h"      /* Hardware access */
#include "debug_interface.h"/* Application debug routines */
#include "gap_service.h"    /* GAP service interface */
#include "battery_service.h"/* Battery service interface */
#include "csr_mesh_light_hw.h"
#include "beacon.h"

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Number of timers used in this application */
#define MAX_TIMERS 1

/* First timeout at which the timer has to fire a callback */
#define TIMER_TIMEOUT1 (1 * SECOND)

/* Subsequent timeout at which the timer has to fire next callback */
#define TIMER_TIMEOUT2 (1 * SECOND)

/* Subsequent timeout at which the timer has to fire next callback */
#define TIMER_TIMEOUT3 (1 * SECOND)

/* My id */
/* #define MYID 3 */

#define true 1
#define false 0

#define PIO_BUTTON      11          /* PIO connected to the button on CSR10xx */
#define PIO_DIR_OUTPUT  TRUE        /* PIO direction configured as output */
#define PIO_DIR_INPUT   FALSE       /* PIO direction configured as input */

/*============================================================================*
 *  Private Data Types
 *============================================================================*/

/* Enumeration to register the known button states */
typedef enum _button_state
{
    button_state_down,       /* Button was pressed */
    button_state_up,         /* Button was released */
    button_state_unknown     /* Button state is unknown */
} BUTTON_STATE_T;

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Current state of button */
static BUTTON_STATE_T g_cur_button_state = button_state_unknown;

/* Declare timer buffer to be managed by firmware library */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_TIMERS];

static uint8 my_id;
static BD_ADDR_T bdaddr;
static uint8 searchFound_flag = 0xFF;
static uint8 previousSearchFound_flag = 0xFF;
static uint8 search_id = 0xFF;
static uint8 packet_id = 0xFF;;
static uint8 source_id = 0xFF;;
static uint8 rssi = 0xFF;


/* SINK id */
static uint8 sink = 0x10;

/* found flag */
static bool found = false;
/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Start timer */
static void startTimer(uint32 timeout, timer_callback_arg handler);

/* Callback after first timeout */
static void timerCallback1(timer_id const id);

/* Callback after second timeout */
static void timerCallback2(timer_id const id);

/* Callback after third timeout */
/* static void timerCallback3(timer_id const id); */

/* Read the current system time and write to UART */
/* static void printCurrentTime(void); */

/* Convert an integer value into an ASCII string and send to the UART */
/* static uint8 writeASCIICodedNumber(uint32 value); */

static void startAdvertising(void);
static void startScanning(void);
static void sinkStopAdvertise(timer_id const id);
static void sinkStartAdvertise(timer_id const id);
static uint16 appRandomDelay(void);
uint8 advData[MAX_ADVERT_PACKET_SIZE];
static BD_ADDR_T  address;

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      appRandomDelay
 *
 *  DESCRIPTION
 *      This function generates a random delay from 0 to 10ms in steps of 50us.
 *
 *  RETURNS
 *      Returns delay in Micro Seconds.
 *
 *---------------------------------------------------------------------------*/
static uint16 appRandomDelay(void)
{
    uint16 rand_num = Random16();
    rand_num = rand_num%201;
    return (rand_num * 50);
}
/*----------------------------------------------------------------------------*
 *  NAME
 *      startTimer
 *
 *  DESCRIPTION
 *      Start a timer
 *
 * PARAMETERS
 *      timeout [in]    Timeout period in seconds
 *      handler [in]    Callback handler for when timer expires
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void startTimer(uint32 timeout, timer_callback_arg handler)
{
    /* Now starting a timer */
    const timer_id tId = TimerCreate(timeout, TRUE, handler);
    
    /* If a timer could not be created, panic to restart the app */
    if (tId == TIMER_INVALID)
    {
        /* DebugWriteString("\r\nFailed to start timer"); */
        
        /* Panic with panic code 0xfe */
        Panic(0xfe);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      timerCallback1
 *
 *  DESCRIPTION
 *      This function is called when the timer created by TimerCreate expires.
 *      It creates a new timer that will expire after the second timer interval.
 *
 * PARAMETERS
 *      id [in]     ID of timer that has expired
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void timerCallback1(timer_id const id)
{
    /* Stop broadcasting */
    LsStartStopAdvertise(FALSE, whitelist_disabled, ls_addr_type_public);
    startScanning();
    /* Now start a new timer for second callback */

    
    startTimer(TIMER_TIMEOUT2 + appRandomDelay(), timerCallback2);    
    
    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      timerCallback2
 *
 *  DESCRIPTION
 *      This function is called when the timer created by TimerCreate expires.
 *      It creates a new timer that will expire after the first timer interval.
 *
 * PARAMETERS
 *      id [in]     ID of timer that has expired
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void timerCallback2(timer_id const id)
{
    /* DebugIfWriteString("Advertising\r\n"); */
    /* Stop scanning */
    LsStartStopScan(FALSE, whitelist_disabled, ls_addr_type_public);  
    startAdvertising();
    /* Now start a new timer for first callback */
    startTimer(TIMER_TIMEOUT1 + appRandomDelay(), timerCallback1);
}

static void sinkStartAdvertise(timer_id const id)
{
    /* DebugIfWriteString("Advertising\r\n"); */
    startAdvertising();
    /* Now start a new timer for first callback */
    startTimer(TIMER_TIMEOUT1 + appRandomDelay(), sinkStopAdvertise);
}

static void sinkStopAdvertise(timer_id const id)
{
    /* DebugIfWriteString("Advertising\r\n"); */
    /* Stop scanning */
    ls_addr_type addressType = ls_addr_type_public;     /* use public address */
     LsStartStopAdvertise(FALSE, whitelist_disabled, addressType);
    startTimer(TIMER_TIMEOUT1 + appRandomDelay(), sinkStartAdvertise);
}
/*
static void timerCallback3(timer_id const id)
{
    found = false;
    LightHardwareSetColor(0, 127, 0);
 
    advData[0] = AD_TYPE_MANUF;


    advData[1] = 0x0A;
    advData[2] = 0x00;
\
    advData[3] = 0xFF;
  
    advData[4] = 0xFF;
  
    advData[5] = 0xFF;
  
    advData[6] = 0xFF;
  
    advData[7] = my_id;
}
*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      printCurrentTime
 *
 *  DESCRIPTION
 *      Read the current system time and write to UART.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
/*
static void printCurrentTime(void)
{
   
    const uint32 now = TimeGet32();
    
    
    DebugIfWriteString("\n\nCurrent system time: ");
    writeASCIICodedNumber(now / MINUTE);
    DebugIfWriteString("m ");
    writeASCIICodedNumber((now % MINUTE)/SECOND);
    DebugIfWriteString("s\r\n");
    
}
*/


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This user application function is called just after a power-on reset
 *      (including after a firmware panic), or after a wakeup from Hibernate or
 *      Dormant sleep states.
 *
 *      At the time this function is called, the last sleep state is not yet
 *      known.
 *
 *      NOTE: this function should only contain code to be executed after a
 *      power-on reset or panic. Code that should also be executed after an
 *      HCI_RESET should instead be placed in the AppInit() function.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppPowerOnReset(void)
{
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This user application function is called after a power-on reset
 *      (including after a firmware panic), after a wakeup from Hibernate or
 *      Dormant sleep states, or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after app_power_on_reset().
 *
 * PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state last_sleep_state)
{
    /* Configure button to be controlled directly */
    PioSetMode(PIO_BUTTON, pio_mode_user);
    
    /* Configure button to be input */
    PioSetDir(PIO_BUTTON, PIO_DIR_INPUT);
    
    /* Set weak pull up on button PIO, in order not to draw too much current
     * while button is pressed
     */
    PioSetPullModes((1UL << PIO_BUTTON), pio_mode_weak_pull_up);

    /* Set the button to generate sys_event_pio_changed when pressed as well
     * as released
     */
    PioSetEventMask((1UL << PIO_BUTTON), pio_event_mode_both);
    
    CSReadBdaddr(&bdaddr);
    /* Initialise communications */
    DebugIfInit();
    DebugIfWriteString("\r\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\r\n");
    my_id = bdaddr.lap;

    GapSetStaticAddress();
    LightHardwareInit();
    TimerInit(MAX_TIMERS, (void *)app_timers);

    /* Report current time */
    /* printCurrentTime(); */
    
     BatteryDataInit();
     
    /* Initialise the application GATT data */
    InitGattData();
    
    /* Reset application hardware data */
    HwDataReset();

    /* Initialise GAP data structure */
    GapDataInit();    
    
    
     startTimer(0, timerCallback1);
    
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcesSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 * PARAMETERS
 *      id   [in]   System event ID
 *      data [in]   Event data
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{
    
    
    
if (id == sys_event_battery_low)
        {
        DebugIfWriteString("sys_event_battery_low");
            /* Battery low event received - notify the connected host. If
             * not connected, the battery level will get notified when
             * device gets connected again
             */
           

        }    
    
     /* If the reported system event is generated by a PIO */
    if (id == sys_event_pio_changed)
    {
        /*DebugIfWriteString("sys_event_pio_changed");
      DebugIfWriteString("\r\n"); */
      
        /* The PIO data is defined by struct pio_changed_data */
        const pio_changed_data *pPioData = (const pio_changed_data *)data;
        
        /* If the PIO event comes from the button */
        if (pPioData->pio_cause & (1UL << PIO_BUTTON))
        {
            bool validStateChange = FALSE;
            
            /* If PIO was HIGH when this event was generated (that means
             * button was released, generating a rising edge causing PIO
             * to go from LOW to HIGH)
             */
            if (pPioData->pio_state & (1UL << PIO_BUTTON))
            {
                /* At this point the button is released */
                g_cur_button_state = button_state_up;
            }
            else
            /* If PIO was LOW when this event was generated (that means
             * button was pressed, generating a falling edge causing
             * PIO to go from HIGH to LOW)
             */
            {
                /* If last recorded state of button was up */
                if (g_cur_button_state == button_state_up)
                {
                    /* This state change is valid */
                    validStateChange = TRUE;
                }

                /* At this point the button has been pressed */
                g_cur_button_state = button_state_down;
            }

            /* If it was a valid button state change */
            if (validStateChange)
            {
                /* Advance LED to next state */
                /* goToNextLedSeq(); */
            }
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event
 *      is received by the system.
 *
 * PARAMETERS
 *      event_code [in]   LM event ID
 *      event_data [in]   LM event data
 *
 * RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
 *----------------------------------------------------------------------------*/
/*bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *event_data)
{
    DebugIfWriteString("\r\nAppProcessLmEvent\r\n");
    return TRUE;
}
*/

void startAdvertising(void)
{
    DebugIfWriteString("Addr: ");
    DebugWriteUint8(WORD_LSB(address.lap));
    DebugIfWriteString("\n ");    
    /* Initialise the application GATT data */
    InitGattData();
    
    /* Reset application hardware data */
    HwDataReset();

    /* Initialise GAP data structure */
    GapDataInit();
    
    
    /* uint16 offset = 0; */
    /* uint8 filler; */
    uint16 advInterval;
    uint8 advPayloadSize;
    ls_addr_type addressType = ls_addr_type_public;     /* use public address */
    
    /* initialise values from User CsKeys */
    
    /* read User key 0 for the payload filler */
    /* filler = (uint8)(CSReadUserKey(0) & 0x00FF); */
    
    /* read User key 1 for the payload size */
    advPayloadSize = (uint8)(CSReadUserKey(1) & 0x00FF);
    
    /* range check */
    if((advPayloadSize < 1) || (advPayloadSize > MAX_ADVERT_PAYLOAD_SIZE))
    {
        /* revert to default payload size */
        advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;
    }
    
    /* read User key 2 for the advertising interval */
    advInterval = CSReadUserKey(2);
    
    /* range check */
    if((advInterval < MIN_ADVERTISING_INTERVAL) ||
       (advInterval > MAX_ADVERTISING_INTERVAL))
    {
        advInterval = MIN_ADVERTISING_INTERVAL;
    }

        /* use random address type */
        addressType = ls_addr_type_public;

        /* generate and set the random address */
        #ifdef USE_STATIC_RANDOM_ADDRESS
        DebugIfWriteString("Setting Random Address! \r\n");
        appSetRandomAddress();
        #endif
    
    GattInit();
    /* set the GAP Broadcaster role */
    GapSetMode(gap_role_broadcaster,
               gap_mode_discover_no,
               gap_mode_connect_no,
               gap_mode_bond_no,
               gap_mode_security_none);
    
    /* clear the existing advertisement data, if any */
    LsStoreAdvScanData(0, NULL, ad_src_advertise);

    /* set the advertisement interval, API accepts the value in microseconds */
    GapSetAdvInterval(advInterval * MILLISECOND, advInterval * MILLISECOND);
    
    /* manufacturer-specific data */
    advData[0] = AD_TYPE_MANUF;
   /* DebugIfWriteUint8(AD_TYPE_MANUF); */
    /* CSR company code, little endian */
    advData[1] = 0x0A;
    advData[2] = 0x00;
    /* Device id*/
    advData[3] = 0xFF;
    /* Search/Found flag */
    advData[4] = 0xE5;
    /* Search Device id */
    advData[5] = 0xDA;
    /* packet id */
    advData[6] = 0x1A;
    /* Source id */
    advData[7] = 0xB0;
    /*
    advData[7] = 0x05;
    advData[8] = 0x06;
    */
    advData[8] = source_id;
    advData[9] = rssi;
   
   if (WORD_LSB(my_id) == WORD_LSB(sink))
    {
        /* Device id*/
    advData[3] = 0xFF;
    /* Search/Found flag */
    advData[4] = 0xE5;
    /* Search Device id */
    advData[5] = 0xDA;
    /* packet id */
    advData[6] = 0x1A;
    /* Source id */
    advData[7] = 0xB0;
    advData[8] = source_id;
    advData[9] = rssi;
    }
   
   if (found == true)
   {
         /* Device id*/
    advData[3] = 0xFF;
    /* Search/Found flag */
    advData[4] = 0xE5;
    /* Search Device id */
    advData[5] = 0xDA;
    /* packet id */
    advData[6] = 0x1A;
    /* Source id */
    advData[7] = 0xB0;
        /* rssi */
     advData[8] = source_id;
    advData[9] = rssi;
              
           DebugIfWriteString(" \r\n");
           
       /* startTimer(TIMER_TIMEOUT3, timerCallback3); */
    }       


    /* store the advertisement data */
    LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);
    
    
    /* scan response data */
    /* uint8 scanresp[17] = {AD_TYPE_MANUF,0x85,0x00,0xFF,0x78,0x56,0x34,0x12, 0x01,0x00,0x00,0x6D,0x77,0xBA,0xDA,0x01}; */
    
    /* Start broadcasting */
    if (WORD_LSB(my_id) == WORD_LSB(sink))
    {
    
    LightHardwareSetLevel(0, 0, 255, 5);
}

void startScanning(void){
    
    address.lap = 10;
    GattInit();

    /* Initialise the application GATT data */
    InitGattData();
    
    /* Reset application hardware data */
    HwDataReset();

    /* Initialise GAP data structure */
    GapDataInit();
    
    GapSetMode(gap_role_observer,
    gap_mode_discover_null, 
    gap_mode_connect_no,
    gap_mode_bond_no,
    gap_mode_security_none);

    GapSetScanInterval(400 * MILLISECOND, 400 * MILLISECOND);
    GapSetScanType(ls_scan_type_passive); 
    
    /* DebugIfWriteString("\r\n");*/
    GapDataInit();
    /*DebugIfWriteString("Scanning\r\n");*/
    LsStartStopScan(TRUE, whitelist_disabled, ls_addr_type_public);
    
    // process META_ADVERTISING_REPORT in AppProcessLmEvent
    /* LightHardwareSetLevel(0, 127, 0, 5); */
    LightHardwareSetLevel(0, 0, 255, 5);
}

bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *p_event_data)
{
        if (event_code==LM_EV_ADVERTISING_REPORT){
       
            LightHardwareSetLevel(0, 0, 255, 5);
            uint16* adv_data2 = (uint16*)p_event_data + sizeof(LM_EV_ADVERTISING_REPORT_T);            
            address = p_event_data->adv_report.data.address;

            if (WORD_LSB(address.lap)==0x16){
           
            uint8 num_uuids;
            uint8 ind = 0;
           /*for(num_uuids=0; num_uuids<num_bytes; num_uuids++)*/
            for(num_uuids=0; num_uuids<10; num_uuids++)
            {
                ind = ind + 1;
                /* DebugWriteUint8(ind);
                DebugIfWriteString(": "); */
                uint8 array[2];
                array[0]=adv_data2[num_uuids + 2] & 0xff;
                array[1]=(adv_data2[num_uuids + 2] >> 8);
                if (ind == 5){
                    /* DebugIfWriteUint8(array[0]); */
                
                }
                if (ind == 9)
                {
                    rssi = p_event_data->adv_report.rssi;
                }
                else if (ind == 8)
                {
                    source_id = array[0];     
                    }
                else if (ind == 7)
                {
                    packet_id = array[0];
                }
                else if (ind == 6)
                {
                    search_id = array[0];
                }
                else if (ind == 5)
                {
                    searchFound_flag = array[0];
                }
                        
                ind = ind + 1;
                if (ind == 9)
                {
                    rssi = p_event_data->adv_report.rssi;
                }
                if (ind == 8)
                {
                    source_id = array[1];                
                    }
                else if (ind == 7)
                {
                    packet_id = array[1];
                }
                else if (ind == 6)
                {
                    search_id = array[1];
                }
                else if (ind == 5)
                {
                    searchFound_flag = array[1];
                }
            }
            
                    DebugIfWriteString("Chair Detected ");
                    LightHardwareSetLevel(0, 0, 255, 5);
                    found = true;
                    rssi=p_event_data->adv_report.rssi;
                    DebugIfWriteString("RSSI: ");
                    rssi = p_event_data->adv_report.rssi;
                    DebugIfWriteUint8(rssi);
                    DebugIfWriteString(" ->>>> ");
                    rssi = rssi >= 0x80   ? (int8) (rssi - 256) : (int8) rssi;
                    DebugIfWriteInt(rssi);
                    DebugIfWriteString("\r\n");
                }
    }
        else {
            DebugIfWriteString("Something happened");
            DebugIfWriteString("\r\n");
        }
        
        if (previousSearchFound_flag != searchFound_flag){
        if (searchFound_flag == 0x02)
                {
        }
        previousSearchFound_flag = searchFound_flag;
    }
    return TRUE;
}