/******************************************************************************
 *  Copyright (c) 2013 - 2016 Qualcomm Technologies International, Ltd.
 *  Part of CSR uEnergy SDK 2.6.0
 *  Application version 2.6.0.0
 *
 *  FILE
 *      gatt_server.c
 *
 *  DESCRIPTION
 *      This file defines a simple implementation of a GATT server.
 *
 ******************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>           /* Functions relating to powering up the device */
#include <types.h>          /* Commonly used type definitions */
#include <timer.h>          /* Chip timer functions */
#include <mem.h>            /* Memory library */
#include <config_store.h>   /* Interface to the Configuration Store */

/* Upper Stack API */
#include <gatt.h>           /* GATT application interface */
#include <ls_app_if.h>      /* Link Supervisor application interface */
#include <gap_app_if.h>     /* GAP application interface */
#include <buf_utils.h>      /* Buffer functions */
#include <security.h>       /* Security Manager application interface */
#include <panic.h>          /* Support for applications to panic */
#include <nvm.h>            /* Access to Non-Volatile Memory */
#include <random.h>         /* Generators for pseudo-random data sequences */

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


/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Maximum number of timers. Up to five timers are required by this application:
 *  
 *  buzzer.c:       buzzer_tid
 *  This file:      con_param_update_tid
 *  This file:      app_tid
 *  This file:      bonding_reattempt_tid (if PAIRING_SUPPORT defined)
 *  hw_access.c:    button_press_tid
 */
#define MAX_APP_TIMERS                 (5)

/* Number of Identity Resolving Keys (IRKs) that application can store */
#define MAX_NUMBER_IRK_STORED          (1)

/* Magic value to check the sanity of Non-Volatile Memory (NVM) region used by
 * the application. This value is unique for each application.
 */
#define NVM_SANITY_MAGIC               (0xABAA)

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_SANITY_WORD         (0)

/* NVM offset for bonded flag */
#define NVM_OFFSET_BONDED_FLAG         (NVM_OFFSET_SANITY_WORD + 1)

/* NVM offset for bonded device Bluetooth address */
#define NVM_OFFSET_BONDED_ADDR         (NVM_OFFSET_BONDED_FLAG + \
                                        sizeof(g_app_data.bonded))

/* NVM offset for diversifier */
#define NVM_OFFSET_SM_DIV              (NVM_OFFSET_BONDED_ADDR + \
                                        sizeof(g_app_data.bonded_bd_addr))

/* NVM offset for IRK */
#define NVM_OFFSET_SM_IRK              (NVM_OFFSET_SM_DIV + \
                                        sizeof(g_app_data.diversifier))

/* Number of words of NVM used by application. Memory used by supported 
 * services is not taken into consideration here.
 */
#define NVM_MAX_APP_MEMORY_WORDS       (NVM_OFFSET_SM_IRK + \
                                        MAX_WORDS_IRK)

/* Slave device is not allowed to transmit another Connection Parameter 
 * Update request till time TGAP(conn_param_timeout). Refer to section 9.3.9.2,
 * Vol 3, Part C of the Core 4.0 BT spec. The application should retry the 
 * 'Connection Parameter Update' procedure after time TGAP(conn_param_timeout)
 * which is 30 seconds.
 */
#define GAP_CONN_PARAM_TIMEOUT          (30 * SECOND)

/*============================================================================*
 *  Private Data types
 *============================================================================*/

/* Application data structure */
typedef struct _APP_DATA_T
{
    /* Current state of application */
    app_state                  state;

    /* TYPED_BD_ADDR_T of the host to which device is connected */
    TYPED_BD_ADDR_T            con_bd_addr;

    /* Track the Connection Identifier (UCID) as Clients connect and
     * disconnect
     */
    uint16                     st_ucid;

    /* Boolean flag to indicate whether the device is bonded */
    bool                       bonded;

    /* TYPED_BD_ADDR_T of the host to which device is bonded */
    TYPED_BD_ADDR_T            bonded_bd_addr;

    /* Diversifier associated with the Long Term Key (LTK) of the bonded
     * device
     */
    uint16                     diversifier;

    /* Timer ID for Connection Parameter Update timer in Connected state */
    timer_id                   con_param_update_tid;

    /* Central Private Address Resolution IRK. Will only be used when
     * central device used resolvable random address.
     */
    uint16                     irk[MAX_WORDS_IRK];

    /* Number of connection parameter update requests made */
    uint8                      num_conn_update_req;

    /* Boolean flag to indicate pairing button press */
    bool                       pairing_button_pressed;

    /* Timer ID for 'UNDIRECTED ADVERTS' and activity on the sensor device like
     * measurements or user intervention in CONNECTED state.
     */
    timer_id                   app_tid;

    /* Boolean flag to indicate whether to set white list with the bonded
     * device. This flag is used in an interim basis while configuring 
     * advertisements.
     */
    bool                       enable_white_list;

#ifdef PAIRING_SUPPORT
    /* Boolean flag to indicate whether encryption is enabled with the bonded
     * host
     */
    bool                       encrypt_enabled;

    /* This timer will be used if the application is already bonded to the 
     * remote host address but the remote device wanted to rebond which we had 
     * declined. In this scenario, we give ample time to the remote device to 
     * encrypt the link using old keys. If remote device doesn't encrypt the 
     * link, we will disconnect the link when this timer expires.
     */
    timer_id                   bonding_reattempt_tid;
#endif /* PAIRING_SUPPORT */

    /* Current connection interval */
    uint16                     conn_interval;

    /* Current slave latency */
    uint16                     conn_latency;

    /* Current connection timeout value */
    uint16                     conn_timeout;
} APP_DATA_T;

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Declare space for application timers */
/*static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];*/

/* Application data instance */
static APP_DATA_T g_app_data;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Initialise application data structure */
static void appDataInit(void);

/* Enable whitelist based advertising */
static void enableWhiteList(void);

#if defined(CONNECTED_IDLE_TIMEOUT_VALUE)
    /* Handle Idle timer expiry in connected states */
    static void appIdleTimerHandler(timer_id tid);

    /* Reset the time for which the application was idle */
    static void resetIdleTimer(void);
#endif /* CONNECTED_IDLE_TIMEOUT_VALUE */

#ifdef PAIRING_SUPPORT
    /* Handle the expiry of the bonding chance timer */
    static void handleBondingChanceTimerExpiry(timer_id tid);
#endif /* PAIRING_SUPPORT */


/* Exit the advertising states */
static void appExitAdvertising(void);

/* Exit the initialisation state */
static void appInitExit(void);

/* Handle advertising timer expiry */
static void appAdvertTimerHandler(timer_id tid);


#ifdef PAIRING_SUPPORT
    /* SM_PAIRING_AUTH_IND signal handler */
    static void handleSignalSmPairingAuthInd(
                    SM_PAIRING_AUTH_IND_T *p_event_data);
    
    /* LM_EV_ENCRYPTION_CHANGE signal handler */
    static void handleSignalLMEncryptionChange(
                    HCI_EV_DATA_ENCRYPTION_CHANGE_T *p_event_data);
#endif /* PAIRING_SUPPORT */






/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      appDataInit
 *
 *  DESCRIPTION
 *      This function is called to initialise the application data structure.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appDataInit(void)
{
    /* Initialise general application timer */
    if (g_app_data.app_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_tid);
        g_app_data.app_tid = TIMER_INVALID;
    }

    /* Reset the pairing button press flag */
    g_app_data.pairing_button_pressed = FALSE;

    /* Initialise the connection parameter update timer */
    if (g_app_data.con_param_update_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.con_param_update_tid);
        g_app_data.con_param_update_tid = TIMER_INVALID;
    }

    /* Initialise the connected client ID */
    g_app_data.st_ucid = GATT_INVALID_UCID;

    /* Initialise white list flag */
    g_app_data.enable_white_list = FALSE;

#ifdef PAIRING_SUPPORT
    /* Initialise link encryption flag */
    g_app_data.encrypt_enabled = FALSE;

    /* Initialise the bonding reattempt timer */
    if (g_app_data.bonding_reattempt_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.bonding_reattempt_tid);
        g_app_data.bonding_reattempt_tid = TIMER_INVALID;
    }
#endif /* PAIRING_SUPPORT */

    /* Reset the connection parameter variables */
    g_app_data.conn_interval = 0;
    g_app_data.conn_latency = 0;
    g_app_data.conn_timeout = 0;

    /* Initialise the application GATT data */
    InitGattData();
    
    /* Reset application hardware data */
    HwDataReset();

    /* Initialise GAP data structure */
    GapDataInit();

    /* Battery Service data initialisation */
    BatteryDataInit();

    /* Call the required service data initialisation APIs from here */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      readPersistentStore
 *
 *  DESCRIPTION
 *      This function is used to initialise and read NVM data.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 *  NAME
 *      enableWhiteList
 *
 *  DESCRIPTION
 *      This function enables white list based advertising.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void enableWhiteList(void)
{
    if(IsDeviceBonded())
    {
        if(!GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr))
        {
            /* Enable white list if the device is bonded and the bonded host 
             * is not using resolvable random address.
             */
            g_app_data.enable_white_list = TRUE;
        }
    }
}

#if defined(CONNECTED_IDLE_TIMEOUT_VALUE)
/*----------------------------------------------------------------------------*
 *  NAME
 *      appIdleTimerHandler
 *
 *  DESCRIPTION
 *      This function is used to handle Idle timer expiry in connected states.
 *      At the expiry of this timer, application shall disconnect with the 
 *      host and shall move to 'APP_DISCONNECTING' state.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appIdleTimerHandler(timer_id tid)
{
    if(tid == g_app_data.app_tid)
    {
        /* Timer has just expired, so mark it as invalid */
        g_app_data.app_tid = TIMER_INVALID;

        /* Handle signal as per current state */
        switch(g_app_data.state)
        {
            case app_state_connected:
            {
                /* Trigger Disconnect and move to app_state_disconnecting 
                 * state 
                 */
                SetState(app_state_disconnecting);
            }
            break;

            default:
                /* Ignore timer in any other state */
            break;
        }

    } /* Else ignore the timer */
}
#endif /* CONNECTED_IDLE_TIMEOUT_VALUE */

#ifdef PAIRING_SUPPORT
/*----------------------------------------------------------------------------*
 *  NAME
 *      handleBondingChanceTimerExpiry
 *
 *  DESCRIPTION
 *      This function is handle the expiry of bonding chance timer.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleBondingChanceTimerExpiry(timer_id tid)
{
    if(g_app_data.bonding_reattempt_tid == tid)
    {
        /* The timer has just expired, so mark it as invalid */
        g_app_data.bonding_reattempt_tid = TIMER_INVALID;
        /* The bonding chance timer has expired. This means the remote has not
         * encrypted the link using old keys. Disconnect the link.
         */
        SetState(app_state_disconnecting);
    }/* Else it may be due to some race condition. Ignore it. */
}
#endif /* PAIRING_SUPPORT */

/*----------------------------------------------------------------------------*
 *  NAME
 *      appExitAdvertising
 *
 *  DESCRIPTION
 *      This function is called while exiting app_state_fast_advertising and
 *      app_state_slow_advertising states.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appExitAdvertising(void)
{
    /* Cancel advertisement timer. Must be valid because timer is active
     * during app_state_fast_advertising and app_state_slow_advertising states.
     */
    TimerDelete(g_app_data.app_tid);
    g_app_data.app_tid = TIMER_INVALID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appAdvertTimerHandler
 *
 *  DESCRIPTION
 *      This function is used to handle Advertisement timer expiry.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appAdvertTimerHandler(timer_id tid)
{
    /* Based upon the timer id, stop on-going advertisements */
    if(g_app_data.app_tid == tid)
    {
        /* Timer has just expired so mark it as invalid */
        g_app_data.app_tid = TIMER_INVALID;

        GattStopAdverts();
    }/* Else ignore timer expiry, could be because of 
      * some race condition */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appInitExit
 *
 *  DESCRIPTION
 *      This function is called upon exiting from app_state_init state. The 
 *      application starts advertising after exiting this state.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appInitExit(void)
{
    if(g_app_data.bonded && 
       !GattIsAddressResolvableRandom(&g_app_data.bonded_bd_addr))
    {
        /* If the device is bonded and the bonded device address is not
         * resolvable random, configure the white list with the bonded 
         * host address.
         */
        if(LsAddWhiteListDevice(&g_app_data.bonded_bd_addr) != ls_err_none)
        {
            ReportPanic(app_panic_add_whitelist);
        }
    }
}

#if defined(CONNECTED_IDLE_TIMEOUT_VALUE)
/*----------------------------------------------------------------------------*
 *  NAME
 *      resetIdleTimer
 *
 *  DESCRIPTION
 *      This function is used to reset the time for which the application was
 *      idle during the connected state.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void resetIdleTimer(void)
{
    /* Delete the Idle timer, if already running */
    if (g_app_data.app_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_tid);
    }

    /* Start the Idle timer again.*/
    g_app_data.app_tid  = TimerCreate(CONNECTED_IDLE_TIMEOUT_VALUE, 
                                    TRUE, appIdleTimerHandler);
}
#endif /* CONNECTED_IDLE_TIMEOUT_VALUE */


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      ReportPanic
 *
 *  DESCRIPTION
 *      This function calls firmware panic routine and gives a single point 
 *      of debugging any application level panics.
 *
 *  PARAMETERS
 *      panic_code [in]         Code to supply to firmware Panic function.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void ReportPanic(app_panic_code panic_code)
{
    /* Raise panic */
    Panic(panic_code);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandleShortButtonPress
 *
 *  DESCRIPTION
 *      This function contains handling of short button press. If connected,
 *      the device disconnects from the connected host else it triggers
 *      advertisements.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandleShortButtonPress(void)
{
    /* Indicate short button press using short beep */
    SoundBuzzer(buzzer_beep_short);

    /* Handle signal as per current state */
    switch(g_app_data.state)
    {
        case app_state_connected:
            /* Disconnect from the connected host */
            SetState(app_state_disconnecting);
            
            /* As per the specification Vendor may choose to initiate the 
             * idle timer which will eventually initiate the disconnect.
             */
             
        break;

        case app_state_idle:
            /* Trigger fast advertisements */
            SetState(app_state_fast_advertising);
        break;

        default:
            /* Ignore in remaining states */
        break;

    }

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      SetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  PARAMETERS
 *      new_state [in]          State to move to
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void SetState(app_state new_state)
{
    /* Check that the new state is not the same as the current state */
    app_state old_state = g_app_data.state;
    
    if (old_state != new_state)
    {
        /* Exit current state */
        switch (old_state)
        {
            case app_state_init:
                appInitExit();
            break;

            case app_state_disconnecting:
                /* Common things to do whenever application exits
                 * app_state_disconnecting state.
                 */

                /* Initialise application and used services data structure 
                 * while exiting Disconnecting state
                 */
                appDataInit();
            break;

            case app_state_fast_advertising:  /* FALLTHROUGH */
            case app_state_slow_advertising:
                /* Common things to do whenever application exits
                 * APP_*_ADVERTISING state.
                 */
                appExitAdvertising();
            break;

            case app_state_connected:
                /* The application may need to maintain the values of some
                 * profile specific data across connections and power cycles.
                 * These values would have changed in 'connected' state. So,
                 * update the values of this data stored in the NVM.
                 */
            break;

            case app_state_idle:
                /* Nothing to do */
            break;

            default:
                /* Nothing to do */
            break;
        }

        /* Set new state */
        g_app_data.state = new_state;

        /* Enter new state */
        switch (new_state)
        {
            case app_state_fast_advertising:
            {
                /* Enable white list if application is bonded to some remote 
                 * device and that device is not using resolvable random 
                 * address.
                 */
                enableWhiteList();
                /* Trigger fast advertisements. */
                GattTriggerFastAdverts(&g_app_data.bonded_bd_addr);

                /* Indicate advertising mode by sounding two short beeps */
                SoundBuzzer(buzzer_beep_twice);
            }
            break;

            case app_state_slow_advertising:
                /* Start slow advertisements */
                GattStartAdverts(&g_app_data.bonded_bd_addr, FALSE);
            break;

            case app_state_idle:
                /* Sound long beep to indicate non-connectable mode */
                SoundBuzzer(buzzer_beep_long);
            break;

            case app_state_connected:
            {
                /* Common things to do whenever application enters
                 * app_state_connected state.
                 */
#ifdef PAIRING_SUPPORT                
                /* Trigger SM Slave Security request only if the remote 
                 * host is not using resolvable random address
                 */
                if(!GattIsAddressResolvableRandom(&g_app_data.con_bd_addr))
                {
                    SMRequestSecurityLevel(&g_app_data.con_bd_addr);
                }
#else /* !PAIRING_SUPPORT */
                /* Update battery status at every connection instance. It may 
                 * not be worth updating timer this often, but this will 
                 * depend upon application requirements 
                 */
                BatteryUpdateLevel(g_app_data.st_ucid);
#endif /* PAIRING_SUPPORT */

#if defined(CONNECTED_IDLE_TIMEOUT_VALUE)
                resetIdleTimer();
#endif
             }
            break;

            case app_state_disconnecting:
                /* Disconnect the link */
                GattDisconnectReq(g_app_data.st_ucid);
            break;

            default:
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetState
 *
 *  DESCRIPTION
 *      This function returns the current state of the application.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Current application state
 *----------------------------------------------------------------------------*/
extern app_state GetState(void)
{
    return g_app_data.state;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsWhiteListEnabled
 *
 *  DESCRIPTION
 *      This function returns whether white list is enabled or not.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      TRUE if white list is enabled, FALSE otherwise.
 *----------------------------------------------------------------------------*/
extern bool IsWhiteListEnabled(void)
{
    return g_app_data.enable_white_list;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandlePairingRemoval
 *
 *  DESCRIPTION
 *      This function contains pairing removal handling.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void HandlePairingRemoval(void)
{

    /* Remove bonding information */

    /* The device will no longer be bonded */
    g_app_data.bonded = FALSE;

    /* Write bonded status to NVM */
    Nvm_Write((uint16*)&g_app_data.bonded, 
              sizeof(g_app_data.bonded), 
              NVM_OFFSET_BONDED_FLAG);


    switch(g_app_data.state)
    {

        case app_state_connected:
        {
            /* Disconnect from the connected host before triggering 
             * advertisements again for any host to connect. Application
             * and services data related to bonding status will get 
             * updated while exiting disconnecting state.
             */
            SetState(app_state_disconnecting);

            /* Reset and clear the white list */
            LsResetWhiteList();
        }
        break;

        case app_state_fast_advertising:
        case app_state_slow_advertising:
        {
            /* Initialise application and services data related to 
             * bonding status
             */
            appDataInit();

            /* Set flag for pairing / bonding removal */
            g_app_data.pairing_button_pressed = TRUE;

            /* Stop advertisements first as it may be making use of the white 
             * list. Once advertisements are stopped, reset the white list
             * and trigger advertisements again for any host to connect.
             */
            GattStopAdverts();
        }
        break;

        case app_state_disconnecting:
        {
            /* Disconnect procedure is on-going, so just reset the white list 
             * and wait for the procedure to complete before triggering 
             * advertisements again for any host to connect. Application
             * and services data related to bonding status will get 
             * updated while exiting disconnecting state.
             */
            LsResetWhiteList();
        }
        break;

        default: /* app_state_init / app_state_idle handling */
        {
            /* Initialise application and services data related to 
             * bonding status
             */
            appDataInit();

            /* Reset and clear the white list */
            LsResetWhiteList();

            /* Start fast undirected advertisements */
            SetState(app_state_fast_advertising);
        }
        break;

    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      StartAdvertTimer
 *
 *  DESCRIPTION
 *      This function starts the advertisement timer.
 *
 *  PARAMETERS
 *      interval [in]           Timer duration, microseconds
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void StartAdvertTimer(uint32 interval)
{
    /* Cancel existing timer, if valid */
    if (g_app_data.app_tid != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_tid);
    }

    /* Start advertisement timer  */
    g_app_data.app_tid = TimerCreate(interval, TRUE, appAdvertTimerHandler);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IsDeviceBonded
 *
 *  DESCRIPTION
 *      This function returns the status whether the connected device is 
 *      bonded or not.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      TRUE if device is bonded, FALSE if not.
 *----------------------------------------------------------------------------*/
extern bool IsDeviceBonded(void)
{
    return g_app_data.bonded;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetConnectionID
 *
 *  DESCRIPTION
 *      This function returns the connection identifier.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Connection identifier.
 *----------------------------------------------------------------------------*/
extern uint16 GetConnectionID(void)
{
    return g_app_data.st_ucid;
}

/*============================================================================*
 *  System Callback Function Implementations
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
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event
 *      is received by the system.
 *
 *  PARAMETERS
 *      event_code [in]         LM event ID
 *      event_data [in]         LM event data
 *
 *  RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
 *----------------------------------------------------------------------------*/






