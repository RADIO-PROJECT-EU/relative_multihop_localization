/******************************************************************************
 *  Copyright (c) 2013 - 2016 Qualcomm Technologies International, Ltd.
 *  Part of CSR uEnergy SDK 2.6.0
 *  Application version 2.6.0.0
 *
 *  FILE
 *      gatt_client.c
 *
 *  DESCRIPTION
 *      This file defines a simple implementation of a GATT client.
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
#include "nvm_access.h"     /* Non-volatile memory access */
#include "gatt_client.h"    /* Interface to this file */
#include "gap_access.h"     /* GAP service interface */
#include "debug_interface.h"/* Application debug routines */
#include "battery_service_data.h"   /* Battery Service interface */
#include "dev_info_service_data.h"  /* Device Info Service interface */

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Maximum number of timers */
#ifdef PAIRING_SUPPORT
    /* 1 - Discovery Procedure and connecting state expiry timer
     * 2 - Bonding timer
     */
    #define MAX_APP_TIMERS                 (2)
#else
    /* 1 - Discovery Procedure and connecting state expiry timer */
    #define MAX_APP_TIMERS                 (1)
#endif /* PAIRING_SUPPORT */

/* This macro defines which key types should be excluded from NVM store */
#define INVALID_KEYS                   (1 << SM_KEY_TYPE_NONE | \
                                        1 << SM_KEY_TYPE_SIGN)

/* Magic value to check the sanity of Non-Volatile Memory (NVM) region used by
 * the application. This value is unique for each application.
 */
#define NVM_SANITY_MAGIC               (0xABAB)

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_SANITY_WORD         (0)

/* Total size required in NVM for each bonded device */
#define NVM_OFFSET_SIZE_EACH_DEV       (sizeof(g_app_data.devices[0].bonded)\
                                        + sizeof(g_app_data.devices[0].keys))

/* Calculate NVM offset for each bonded device */
#define NVM_OFFSET_DEV_NUM(x)          ((x) * NVM_OFFSET_SIZE_EACH_DEV)

/* NVM offset for bonded flag */
#define NVM_OFFSET_BONDED_FLAG(x)      (NVM_OFFSET_SANITY_WORD + 1 + \
                                               NVM_OFFSET_DEV_NUM(x))

/* NVM offset for SM Keys */
#define NVM_OFFSET_SM_KEYS(x)          (NVM_OFFSET_BONDED_FLAG(x) + \
                                        sizeof(g_app_data.devices[(x)].bonded))

/* Timer value for starting the Discovery Procedure once the connection is 
 * established. During this time pairing is initiated and completed, if pairing
 * is supported by the application or initiated by the peer device.
 */
#define DISCOVERY_START_TIMER           (300 * MILLISECOND)

/* Timer value for starting the Pairing Procedure once the connection is 
 * established. This has been done to facilitate completion of any GATT
 * procedure requiring the devices to be paired.
 */
#define PAIRING_TIMER_VALUE             (150 * MILLISECOND)

/* Maximum expected time for a connection to be established. */
#define CONNECTING_STATE_EXPIRY_TIMER   (15 * SECOND)

/*============================================================================*
 *  Private Data types
 *============================================================================*/

/* Application data structure */
typedef struct _APP_DATA_T
{
    /* Connected devices */
    DEVICE_T                   devices[MAX_CONNECTED_DEVICES];

    /* Application timer ID */
    timer_id                   app_timer;

#ifdef PAIRING_SUPPORT
    /* Application bonding ID */
    timer_id                   bonding_timer;
#endif /* PAIRING_SUPPORT */

    /* Currently connected device */
    uint16                     dev_num;

    /* Offset to NVM data for the current device */
    uint16                     nvm_dev_num;

    /* Number of connected devices */
    uint16                     num_conn;
} APP_DATA_T;

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* List of supported services' callback function tables */
static SERVICE_FUNC_POINTERS_T *g_supported_services[] = {
    &BatteryServiceFuncStore,   /* Battery Service */
    &DeviceInfoServiceFuncStore /* Device Information Service */
};
                                        
/* Declare space for application timers */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];

/* Application data instance */
static APP_DATA_T g_app_data;

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Initiate the scanning process for the connected device */
static void appStartScan(void);

/* Initialise application data structure */
static void appDataInit(void);

/* Check and read if the NVM data contains the specified device */
static void checkPersistentStore(uint16 *nvmDevNum, TYPED_BD_ADDR_T bdAddress);

/* Initialise and read NVM data for each device */
static void readPersistentStore(uint16 dev_num, uint16 nvm_dev_num);

/* Store the NVM data. readPersistent store must have been called at least once
 * before calling this function.
 */
static void storeNvmData(void);

/* Exit the initialisation state */
static void appInitExit(uint16 dev);

/* Start the Discovery Procedure timer */
static void appStartDiscoveryProcedure(uint16 dev);

/* Start the Discovery Procedure */
static void appStartDiscoveryTimerExpiry(timer_id tid);

/* Request connection parameter update */
static void requestConnParamUpdate(uint16 dev);

/* Handle connection timeout */
static void appConnectingStateTimerExpiry(timer_id tid);

/* Find the device from the given connection handle */
static uint16 findDeviceByHciHandle(hci_connection_handle_t handle);

#ifdef PAIRING_SUPPORT
    /* Start the Pairing Procedure */
    static void appPairingTimerHandlerExpiry(timer_id tid);
#endif /* PAIRING_SUPPORT */

/* LM_EV_CONNECTION_COMPLETE signal handler */
static void handleSignalLmEvConnectionComplete(
                       HCI_EV_DATA_ULP_CONNECTION_COMPLETE_T *p_event_data);

/* GATT_CONNECT_CFM signal handler */
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T* p_event_data);

/* SM_KEY_REQUEST_IND signal handler */
static void handleSignalSmKeyRequestInd(SM_KEY_REQUEST_IND_T * p_event_data);

/* SM_KEYS_IND signal handler */
static void handleSignalSmKeysInd(SM_KEYS_IND_T *p_event_data);

/* SM_SIMPLE_PAIRING_COMPLETE_IND signal handler */
static void handleSignalSmSimplePairingCompleteInd(
                    SM_SIMPLE_PAIRING_COMPLETE_IND_T *p_event_data);

/* LM_EV_DISCONNECTION_COMPLETE signal handler */
static void handleSignalLmDisconnectComplete(
                    HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data);

/* GATT_CANCEL_CONNECT_CFM signal handler */
static void handleSignalGattCancelConnectCfm(void);

/* LS_CONNECTION_PARAM_UPDATE_CFM signal handler */
static void handleSignalLsConnectionParamUpdateCfm(
                                LS_CONNECTION_PARAM_UPDATE_CFM_T *p_event_data);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      appStartScan
 *
 *  DESCRIPTION
 *      Initiates the scanning process for the connected device
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *---------------------------------------------------------------------------*/
static void appStartScan(void)
{
    /* Number of services listed in g_supported_services */
    const uint16 size = sizeof(g_supported_services) /
                        sizeof(SERVICE_FUNC_POINTERS_T *);

    /* Start scanning for Servers advertising any supported service */
#ifdef FILTER_DEVICE_BY_SERVICE
    GattStartScan(size, g_supported_services, TRUE);
#else
    GattStartScan(size, g_supported_services, FALSE);
#endif /* FILTER_DEVICE_BY_SERVICE */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appDataInit
 *
 *  DESCRIPTION
 *      This function is called to initialise the application data structure
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *---------------------------------------------------------------------------*/
static void appDataInit(void)
{
    /* Initialise general application timer */
    if (g_app_data.app_timer != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_timer);
        g_app_data.app_timer = TIMER_INVALID;
    }

#ifdef PAIRING_SUPPORT
    /* Initialise bonding timer */
    if (g_app_data.bonding_timer != TIMER_INVALID)
    {
        TimerDelete(g_app_data.bonding_timer);
        g_app_data.bonding_timer = TIMER_INVALID;
    }
#endif /* PAIRING_SUPPORT */

    /* Initialise the application GATT data. */
    InitGattData();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      checkPersistentStore
 *
 *  DESCRIPTION
 *      Check and read if the NVM data contains the specified device.
 *
 *  PARAMETERS
 *      nvmDevNum [out]         Device number
 *      bdAddress [in]          Bluetooth address of device to find in NVM
 *
 *  RETURNS
 *      Nothing
 *---------------------------------------------------------------------------*/
static void checkPersistentStore(uint16 *nvmDevNum, TYPED_BD_ADDR_T bdAddress)
{
    uint16 dev = MAX_BONDED_DEVICES;    /* Found device number */
    uint16 nvm_sanity = 0xffff;         /* NVM sanity magic number */

    /* Check whether the NVM has been initialised yet by looking for the magic
     * number assigned to this application.
     */
    Nvm_Read(&nvm_sanity, 
             sizeof(nvm_sanity),
             NVM_OFFSET_SANITY_WORD);

    if (nvm_sanity == NVM_SANITY_MAGIC)
    {
        /* Search the list of bonded devices stored in NVM for the supplied
         * Bluetooth address
         */
        for (dev = 0; dev < MAX_BONDED_DEVICES; dev++)
        {
            SM_KEYSET_T keys;
            TYPED_BD_ADDR_T addr;

            Nvm_Read((uint16*) &keys, 
                       sizeof(SM_KEYSET_T),
                       NVM_OFFSET_SM_KEYS(dev));
            
            addr = keys.id_addr;
            if(!MemCmp(&addr, 
                       &bdAddress, 
                       sizeof(TYPED_BD_ADDR_T)))
            {
                /* Device matched */
                break;
            }
        }
    }

    *nvmDevNum = dev;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      readPersistentStore
 *
 *  DESCRIPTION
 *      Initialise and read NVM data.
 *
 *  PARAMETERS
 *      dev [in]                Device number
 *      nvm_dev_num [in]        Index to NVM data for specified device
 *
 *  RETURNS
 *      Nothing
 *---------------------------------------------------------------------------*/
static void readPersistentStore(uint16 dev, uint16 nvm_dev_num)
{
    uint16 nvm_sanity = 0xffff;         /* NVM sanity magic number */

    /* Read persistent storage to know if the device was last bonded 
     * to another device 
     */

    Nvm_Read(&nvm_sanity, 
             sizeof(nvm_sanity), 
             NVM_OFFSET_SANITY_WORD);

    if(nvm_sanity == NVM_SANITY_MAGIC)
    {
        if(dev == MAX_CONNECTED_DEVICES ||
           nvm_dev_num == MAX_BONDED_DEVICES)
        {
            /* The NVM was initialised in its previous run and the application 
             * is coming up again after a reset cycle. Do not store the bonding
             * information at this time
             */
            return;
        }

        /* Read Bonded Flag from NVM */
        Nvm_Read((uint16*)&g_app_data.devices[dev].bonded,
                  sizeof(g_app_data.devices[dev].bonded),
                  NVM_OFFSET_BONDED_FLAG(nvm_dev_num));

        if(g_app_data.devices[dev].bonded)
        {
            /* Bonded Host Typed Bluetooth Address will only be stored if bonded
             * flag is set to TRUE. Read last bonded device address.
             */

            /* Link keys will only be stored only be stored if bonded flag
             * is set to TRUE. Read the link keys from the persistence store.
             */
            Nvm_Read((uint16*)&g_app_data.devices[dev].keys, 
                       sizeof(g_app_data.devices[dev].keys),
                       NVM_OFFSET_SM_KEYS(nvm_dev_num));

            /* Read last bonded device address */
            MemCopy(&g_app_data.devices[dev].address, 
                    &g_app_data.devices[dev].keys.id_addr,
                    sizeof(TYPED_BD_ADDR_T));
        }
        else /* Case when we have only written NVM_SANITY_MAGIC to NVM but 
              * didn't get bonded to any host in the last powered session
              */
        {
            /* Any initialisation can be done here for non-bonded devices */
        }
    }
    else /* NVM Sanity check failed means either the device is being brought up 
          * for the first time or NVM has been corrupted in which case discard
          * the data and start fresh.
          */
    {
        nvm_sanity = NVM_SANITY_MAGIC;

        /* Write NVM Sanity word to the NVM */
        Nvm_Write(&nvm_sanity, 
                  sizeof(nvm_sanity), 
                  NVM_OFFSET_SANITY_WORD);

        if(dev == MAX_CONNECTED_DEVICES &&
           nvm_dev_num == MAX_BONDED_DEVICES)
        {
            for(dev = 0; dev < MAX_CONNECTED_DEVICES; dev++)
            {
                /* Initialise bonded device flag */
                g_app_data.devices[dev].bonded = FALSE;
        
                /* Store bonded device flag in NVM */
                Nvm_Write((uint16 *)&g_app_data.devices[dev].bonded, 
                           sizeof(g_app_data.devices[dev].bonded), 
                          NVM_OFFSET_BONDED_FLAG(dev));
            }
            
            for (; dev < MAX_BONDED_DEVICES; dev++)
            {
                /* If NVM has support for more than MAX_CONNECTED_DEVICES
                 * devices, initialise bonded device flag in NVM using the
                 * first device as a template.
                 */
                Nvm_Write((uint16 *)&g_app_data.devices[0].bonded, 
                           sizeof(g_app_data.devices[0].bonded), 
                          NVM_OFFSET_BONDED_FLAG(dev));
            }
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      storeNvmData
 *
 *  DESCRIPTION
 *      Store the NVM data. Should only be called after readPersistentStore has
 *      been called at least once.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *---------------------------------------------------------------------------*/
static void storeNvmData(void)
{
    uint16 nvm_sanity = 0xffff;         /* NVM sanity magic number */
    uint16 dev = 0;                     /* Device number */
    bool bondedFlag = FALSE;            /* Whether device is bonded */
    
    /* Check NVM to see whether the currently connected device has already
     * bonded with the Client. If it has then we assume the same keys are
     * used again.
     */
    checkPersistentStore(&dev, 
                         g_app_data.devices[(g_app_data.dev_num)].address);

    if(dev != MAX_BONDED_DEVICES)
    {
        /* Device data already exists in NVM. Implies device is already
         * paired.
         */
        if(dev != g_app_data.nvm_dev_num)
        {
            /* Update the offset to the device's bonding data in NVM */
            g_app_data.nvm_dev_num = dev;
        }

        /* Do not store the data again */
        return;
    }

    Nvm_Read(&nvm_sanity, 
             sizeof(nvm_sanity), 
             NVM_OFFSET_SANITY_WORD);

    if(nvm_sanity == NVM_SANITY_MAGIC)
    {
        /* If pairing data for the current device is not already stored in NVM,
         * then look for the first free slot in NVM to store the data in
         */
        for(dev = 0; dev < MAX_BONDED_DEVICES; dev++)
        {
            Nvm_Read((uint16*)&bondedFlag,
                      sizeof(bondedFlag),
                      NVM_OFFSET_BONDED_FLAG(dev));
            if(!bondedFlag)
            {
                g_app_data.nvm_dev_num = dev;
                break;
            }
        }

        if(dev == MAX_BONDED_DEVICES)
        {
            /* If the NVM has no room to store new bonded devices, overwrite
             * the last entry in the list.
             *
             * It may be preferrable to reject the pairing request if the list
             * is full instead.
             */
            g_app_data.nvm_dev_num = MAX_BONDED_DEVICES - 1;
        }

        /* Store the bonded flag */
        Nvm_Write((uint16*)&g_app_data.devices[(g_app_data.dev_num)].bonded,
                  sizeof(g_app_data.devices[(g_app_data.dev_num)].bonded),
                  NVM_OFFSET_BONDED_FLAG(g_app_data.nvm_dev_num));

        /* Store the Link keys */
        Nvm_Write((uint16*)&g_app_data.devices[(g_app_data.dev_num)].keys, 
                   sizeof(g_app_data.devices[(g_app_data.dev_num)].keys),
                   NVM_OFFSET_SM_KEYS(g_app_data.nvm_dev_num));
    }
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
 *      dev [in]                Device number
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appInitExit(uint16 dev)
{
    /* Initialise data stored for this device */
    MemSet(&g_app_data.devices[dev], 0x0, sizeof(DEVICE_T));

    g_app_data.devices[dev].connectHandle = GATT_INVALID_UCID;
    g_app_data.devices[dev].hciHandle     = GATT_INVALID_UCID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appStartDiscoveryProcedure
 *
 *  DESCRIPTION
 *      Start the Discovery Procedure.
 *
 *  PARAMETERS
 *      dev [in]                Device number
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appStartDiscoveryProcedure(uint16 dev)
{
    /* Enter the discovering state */
    SetState(dev, app_state_discovering);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appStartDiscoveryTimerExpiry
 *
 *  DESCRIPTION
 *      This function is used to handle Discovery timer expiry.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appStartDiscoveryTimerExpiry(timer_id tid)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;
    /* Connection handle */
    const uint16 connectHandle = g_app_data.devices[dev].connectHandle;

    if(tid == g_app_data.app_timer)
    {
        /* Timer has just expired, so mark it as invalid */
        g_app_data.app_timer = TIMER_INVALID;

        /* Start discovering the connected device's GATT database */
        if(!GattDiscoverRemoteDatabase(connectHandle))
        {
            /* No supported services found or Discovery Procedure failed */
            
            /* Disconnect the device */
            SetState(dev, app_state_disconnecting);
        }
    }
    /* Else it may be because of some race condition. Ignore it */
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      requestConnParamUpdate
 *
 *  DESCRIPTION
 *      This function is used to update the connection parameters to reduce
 *      current consumption after the Discovery Procedure completes.
 *
 *  PARAMETERS
 *      dev [in]                Number of device to request connection parameter
 *                              update from
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void requestConnParamUpdate(uint16 dev)
{
    /* New connection parameters */
    ble_con_params app_pref_conn_param = 
                {
                    PREFERRED_MIN_CON_INTERVAL,
                    PREFERRED_MAX_CON_INTERVAL,
                    PREFERRED_RUNNING_SLAVE_LATENCY,
                    PREFERRED_SUPERVISION_TIMEOUT
                };
    
    /* Send a connection parameter update */
    if(LsConnectionParamUpdateReq(&g_app_data.devices[dev].address, 
                                  &app_pref_conn_param))
    {
        ReportPanic(app_panic_con_param_update);
    }
    
    /* When the connection parameters have been updated the firmware will issue
     * a LS_CONNECTION_PARAM_UPDATE_CFM event, which causes this application to
     * move to the app_state_configured state.
     */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appConnectingStateTimerExpiry
 *
 *  DESCRIPTION
 *      This function issues the GATT cancel connect request if the device is in
 *      this state for a long time
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appConnectingStateTimerExpiry(timer_id tid)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;

    if(tid == g_app_data.app_timer)
    {
        /* Timer has just expired, so mark it as invalid */
        g_app_data.app_timer = TIMER_INVALID;

        /* If we're still in the connecting state, cancel the connection */
        if(g_app_data.devices[dev].state == app_state_connecting)
        {
            GattCancelConnectReq();
        }
    }
    /* Else it may be because of some race condition. Ignore it */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      findDeviceByHciHandle
 *
 *  DESCRIPTION
 *      This function finds the device in g_app_data by the given connection
 *      handle.
 *
 *  PARAMETERS
 *      handle [in]             Connection handle of device to find
 *
 *  RETURNS
 *      Corresponding device number.
 *----------------------------------------------------------------------------*/
static uint16 findDeviceByHciHandle(hci_connection_handle_t handle)
{
    uint16 dev_num;             /* Number of connected device */

    /* Browse through all devices */
    for(dev_num = 0; dev_num < MAX_CONNECTED_DEVICES; dev_num++)
    {
        if(g_app_data.devices[dev_num].hciHandle == handle)
            break;
    }
    
    return dev_num;
}

#ifdef PAIRING_SUPPORT
/*----------------------------------------------------------------------------*
 *  NAME
 *      appPairingTimerHandlerExpiry
 *
 *  DESCRIPTION
 *      This function handles the expiry of the pairing timer.
 *
 *  PARAMETERS
 *      tid [in]                ID of timer that has expired
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void appPairingTimerHandlerExpiry(timer_id tid)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;

    if(g_app_data.bonding_timer == tid)
    {
        /* Timer has just expired, so mark it as invalid */
        g_app_data.bonding_timer = TIMER_INVALID;

        /* The bonding chance timer has expired. This means the remote has not
         * encrypted the link using old keys or pairing was not initiated.
         * Try initiating pairing request.
         */

        /* Handling signal as per current state */
        switch(g_app_data.devices[dev].state)
        {
            case app_state_connected:
            case app_state_discovering:
            case app_state_configured:
            {
                /* Initiate pairing */
                if(!GattIsAddressResolvableRandom(&g_app_data.devices[dev].address))
                {
                    SMRequestSecurityLevel(&g_app_data.devices[dev].address);
                }
            }
            break;

            default:
                /* Ignore timer in any other state */
            break;
        }
    }/* Else it may be due to some race condition. Ignore it. */
}
#endif /* PAIRING_SUPPORT */
        
/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalLmEvConnectionComplete
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_COMPLETE.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by LM_EV_CONNECTION_COMPLETE
 *                              signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalLmEvConnectionComplete(
                        HCI_EV_DATA_ULP_CONNECTION_COMPLETE_T *p_event_data)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;

    if(p_event_data->status == HCI_SUCCESS)
    {
        /* Connection complete */

        /* Connected device address is known, compare it to confirm */
        if(MemCmp(&g_app_data.devices[dev].address.addr, 
                  &p_event_data->peer_address, 
                  sizeof(BD_ADDR_T)) ||
           MemCmp(&g_app_data.devices[dev].address.type,
                  &p_event_data->peer_address_type,
                  sizeof(p_event_data->peer_address_type)) ||
           (p_event_data -> role == 0x1))             /* Master role */
        {
            /* Device address does not match, continue scanning */
            
            g_app_data.devices[dev].connected = FALSE;
            
            /* Restart the scanning */
            SetState(dev, app_state_scanning);
            
            return;
        }

        /* Increase the number of connections */
        g_app_data.num_conn++;

        /* Store the device details */
        g_app_data.devices[dev].connected = TRUE;

        g_app_data.devices[dev].hciHandle = p_event_data->connection_handle;


        DebugIfWriteString("\r\n*** Connected to ");
        DebugIfWriteBdAddress(&g_app_data.devices[dev].address);
        DebugIfWriteString(" conn params (");
        DebugIfWriteUint16(p_event_data->conn_interval);
        DebugIfWriteString(" ");
        DebugIfWriteUint16(p_event_data->conn_latency);
        DebugIfWriteString(" ");
        DebugIfWriteUint16(p_event_data->supervision_timeout);
        DebugIfWriteString(")\r\n");
        
        /* Nothing else to do till GATT_CONNECT_CFM event is received */
    }
    else
    {
        g_app_data.devices[dev].connected = FALSE;

        DebugIfWriteString("\r\n*** Failed to connect to ");
        DebugIfWriteBdAddress(&g_app_data.devices[dev].address);
        DebugIfWriteString(" (HCI error code: 0x");
        DebugIfWriteUint16(p_event_data->status);
        DebugIfWriteString(")\r\n");

        /* Restart the scanning */
        SetState(dev, app_state_scanning);
    }
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalGattConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CONNECT_CFM.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by GATT_CONNECT_CFM signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T *p_event_data)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;

    if(p_event_data->result == sys_status_success && /* Connection successful */
       /* Compare the address and its type */
       !(MemCmp(&g_app_data.devices[dev].address, 
                  &p_event_data->bd_addr, 
                  sizeof(TYPED_BD_ADDR_T))) &&
        /* Check if the device is connected */
        (g_app_data.devices[dev].connected))
    {

        /* Store connection handle */
        g_app_data.devices[dev].connectHandle = p_event_data->cid;

        DebugIfWriteString("Connected to ");
        DebugIfWriteBdAddress(&p_event_data->bd_addr);
        DebugIfWriteString(" (");
        DebugIfWriteUint16(p_event_data->cid);
        DebugIfWriteString(")\r\n");

        checkPersistentStore(&g_app_data.nvm_dev_num,
                             p_event_data->bd_addr);

        if(g_app_data.nvm_dev_num < MAX_BONDED_DEVICES)
        {
            /* Read Persistent Store data and store it*/
            readPersistentStore(g_app_data.dev_num,
                                g_app_data.nvm_dev_num);
        }

       SetState(dev, app_state_connected);
    }
    else
    {
        /* Connection failed - remove the device from the list */
        g_app_data.devices[dev].connected = FALSE;

        DebugIfWriteString("Failed to connect to ");
        DebugIfWriteBdAddress(&p_event_data->bd_addr);
        DebugIfWriteString("\r\n");

        /* Restart scanning */
        SetState(dev, app_state_scanning);
    }
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalSmKeyRequestInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_KEY_REQUEST_IND and passes the
 *      keys to the Security Manager, if previously paired and new keys have
 *      not been requested.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_KEY_REQUEST_IND signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmKeyRequestInd(SM_KEY_REQUEST_IND_T *p_event_data)
{
    /* Keys stored for the peer device, if previously paired */
    SM_KEYSET_T *keys = NULL;
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;

    if(g_app_data.devices[dev].bonded && 
       !g_app_data.devices[dev].requestNewKeys)
    {
        /* If the device is bonded, and new keys have not been requested, use
         * the valid keys fetched from NVM
         */
        keys = &g_app_data.devices[dev].keys;
    }

    /* Pass the keys to the SM */
    SMKeyRequestResponse(&g_app_data.devices[dev].address, keys);
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalSmKeysInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_KEYS_IND.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_KEYS_IND signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmKeysInd(SM_KEYS_IND_T *p_event_data)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;

    if(p_event_data->keys != NULL && /* Valid pointer */
       !(p_event_data->keys->keys_present & (uint16)INVALID_KEYS) &&
       (p_event_data->keys->keys_present & (uint16)(1 << SM_BD_ADDR)) &&
       ((g_app_data.devices[dev].bonded != TRUE) ||
       (g_app_data.devices[dev].requestNewKeys == TRUE)))
    {
        /* Store the new keys in application data structure */
        MemCopy(&g_app_data.devices[dev].keys, 
                p_event_data->keys, 
                sizeof(SM_KEYSET_T));

        if(g_app_data.devices[dev].requestNewKeys == TRUE)
        {
            /* Store the new keys in NVM */
            Nvm_Write((uint16*)&g_app_data.devices[(dev)].keys, 
                       sizeof(g_app_data.devices[(dev)].keys),
                       NVM_OFFSET_SM_KEYS(g_app_data.nvm_dev_num));
        }
    }
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalSmSimplePairingCompleteInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_SIMPLE_PAIRING_COMPLETE_IND.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by SM_SIMPLE_PAIRING_COMPLETE_IND
 *                              signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalSmSimplePairingCompleteInd(
                                 SM_SIMPLE_PAIRING_COMPLETE_IND_T *p_event_data)
{
    /* Device Number - to improve readability */
    const uint16 dev = g_app_data.dev_num;

    /* Handling signal as per current state */
    switch(g_app_data.devices[dev].state)
    {
        case app_state_discovering:
        case app_state_connected:
        case app_state_configured:
        {
            if(p_event_data->status == sys_status_success)
            {
                if(p_event_data->security_level == gap_mode_security_unauthenticate)
                {
#ifdef PAIRING_SUPPORT
                    /* Make sure pairing is not requested again by deleting the 
                     * timer
                     */
                    if (g_app_data.bonding_timer != TIMER_INVALID)
                    {
                        TimerDelete(g_app_data.bonding_timer);
                        g_app_data.bonding_timer = TIMER_INVALID;
                    }
#endif /* PAIRING_SUPPORT */

                    DebugIfWriteString("\r\n*** Pairing Completed ");
                    DebugIfWriteBdAddress(&p_event_data->bd_addr);
                    DebugIfWriteString("\r\n");

                    if(GattServiceIncomplete() && 
                       ((!g_app_data.devices[dev].bonded) ||
                       g_app_data.devices[(dev)].encryptAgain))
                        /* Make sure that the device was initially not bonded 
                         * and device is not being re-paired
                         */
                    {
                        /* Implies the Discovery Procedure is still
                         * incomplete
                         */
                        GattInitServiceCompletion(dev, 
                                     g_app_data.devices[dev].connectHandle);
                    }
                    else if(GattPairingInitiated()) 
                    {
                        /* Pairing was initiated due to insufficient
                         * authentication/authorisation. So continue the
                         * Discovery Procedure from where it left off
                         */
                        GattInitiateProcedureAgain(dev);
                    }

                    g_app_data.devices[dev].requestNewKeys = FALSE;

                    g_app_data.devices[dev].encryptAgain = FALSE;

                    /* If the keys supplied during pairing are valid, then
                     * update the keys in NVM and record that the device is
                     * bonded.
                     */
                    if (!MemCmp(&p_event_data->bd_addr,
                                &g_app_data.devices[dev].keys.id_addr,
                                sizeof(TYPED_BD_ADDR_T)))
                    {
                        /* Store bonded host information in NVM. This includes
                         * application and service specific information
                         */
                        g_app_data.devices[dev].bonded = TRUE;
                        storeNvmData();
                    }
                }                
            }
            else if(p_event_data->status == HCI_ERROR_KEY_MISSING)
            /* One of the error codes reported. Refer to Vol 2, Part D,
             * section 2.6 "PIN OR KEY MISSING" for more details
             */
            {
                /* Bonded flag will be updated again in DeviceFound */

                if(g_app_data.devices[(dev)].bonded) /* Device already bonded */
                {
                    g_app_data.devices[(dev)].requestNewKeys = TRUE;

                    g_app_data.devices[(dev)].encryptAgain = TRUE;

#ifdef PAIRING_SUPPORT

                    /* Initiate pairing */
                    StartBonding();

                    DebugIfWriteString("\r\n*** Request pairing again ");
                    DebugIfWriteBdAddress(&p_event_data->bd_addr);
                    DebugIfWriteString("\r\n");

#else /* !PAIRING_SUPPORT */

                    /* Disconnect the link */
                    SetState(dev, app_state_disconnecting);

                    DebugIfWriteString("\r\n*** Disconnect - PIN/KEY missing");
                    DebugIfWriteString(" found and pairing not supported");

                    /* Print the Bluetooth address */
                    DebugIfWriteString("\r\n*** BD Address - ");
                    DebugIfWriteBdAddress(&p_event_data->bd_addr);
                    
                    DebugIfWriteString("\r\n");

#endif /* PAIRING_SUPPORT */
                }
                else
                {
                    /* Bonded flag is false - update the NVM */
                    Nvm_Write((uint16*)&g_app_data.devices[dev].bonded,
                              sizeof(g_app_data.devices[dev].bonded),
                              NVM_OFFSET_BONDED_FLAG(g_app_data.nvm_dev_num));

                    /* Disconnect the device */
                    SetState(dev, app_state_disconnecting);
                
                    DebugIfWriteString("\r\n*** Disconnecting the ");
                    DebugIfWriteBdAddress(&p_event_data->bd_addr);
                    DebugIfWriteString("\r\n");
                }
            }
            else if (p_event_data->status == sm_status_pairing_not_supported)
            {
                DebugIfWriteString("\r\n*** Device ");
                DebugIfWriteBdAddress(&p_event_data->bd_addr);
                DebugIfWriteString(" Already bonded.");
                DebugIfWriteString("\r\n*** Remove pairing to proceed.");
                DebugIfWriteString("\r\n");
            }
        }
        break;

        default:
        break;
    }
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalLmDisconnectComplete
 *
 *  DESCRIPTION
 *      This function handles LM_EV_DISCONNECTION_COMPLETE event which is
 *      received at the completion of disconnect procedure triggered either by 
 *      the device or remote host or because of link loss
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by LM_EV_DISCONNECTION_COMPLETE
 *                              signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalLmDisconnectComplete(
                HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data)
{
    uint16 dev_discon;              /* Number of disconnected device */
    uint16 dev;                     /* Loop counter */
    bool initiateScanning;          /* Whether to start scanning */
    
    /* Find device number of disconnected device */
    dev_discon = findDeviceByHciHandle(p_event_data->handle);

    if(dev_discon < MAX_CONNECTED_DEVICES)
    {
        DebugIfWriteString("\r\n*** Disconnected from ");
        DebugIfWriteBdAddress(&g_app_data.devices[dev_discon].address);

        /* Reset all the service data, connected/discovered for this device */
        GattResetAllServices(dev_discon);

        /* Reset the data in the device record */
        MemSet(&g_app_data.devices[dev_discon], 0x0, sizeof(DEVICE_T));

        g_app_data.devices[dev_discon].connectHandle = GATT_INVALID_UCID;
        g_app_data.devices[dev_discon].hciHandle = GATT_INVALID_UCID;

        /* Decrease the number of connected peripheral devices */
        if(g_app_data.num_conn)
           g_app_data.num_conn--;

        /* Check the state of all the other devices to see whether scanning
         * should be initiated for the disconnected device number
         */
        for (dev = 0, initiateScanning = TRUE;
             (dev < MAX_CONNECTED_DEVICES) && initiateScanning;
             dev++)
        {
            if(dev != dev_discon)
            {
                switch(g_app_data.devices[dev].state)
                {
                    case app_state_connecting:
                    case app_state_scanning:
                    case app_state_connected:
                    case app_state_discovering:
                    {
                        /* As one of the devices is already in one of the above 
                         * states, we cannot initialise scanning for the
                         * disconnected device number.
                         */
                        initiateScanning = FALSE;
                    }
                    break;

                    case app_state_init:
                    case app_state_configured:
                    {
                        /* Safe to start scanning for the disconnected device
                         * number.
                         */
                    }
                    break;

                    default:
                    {
                        /* Control should never come here */
                        ReportPanic(app_panic_invalid_state);
                    }
                    break;
                }
            }
        }
        
        if(initiateScanning)
        {
            /* No other device is being configured, so initiate scanning on this
             * device number.
             */
            SetState(dev_discon, app_state_scanning);
        }
        else
        {
            /* Move to the init state on this device number, as one of the
             * other devices is still being configured.
             */
            SetState(dev_discon, app_state_init);
        }
    }
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalGattCancelConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CANCEL_CONNECT_CFM.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by GATT_CANCEL_CONNECT_CFM signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalGattCancelConnectCfm(void)
{
    /* This event was received after the application was in app_state_connecting
     * for too long, so move back to the app_state_scanning state.
     */
    SetState(g_app_data.dev_num, app_state_scanning);
}

/*---------------------------------------------------------------------------
 *  NAME
 *      handleSignalLsConnectionParamUpdateCfm
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_CFM
 *      received when the connection parameter update requested by the Master
 *      completes.
 *
 *  PARAMETERS
 *      p_event_data [in]       Data supplied by LS_CONNECTION_PARAM_UPDATE_CFM
 *                              signal
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void handleSignalLsConnectionParamUpdateCfm(
                                 LS_CONNECTION_PARAM_UPDATE_CFM_T *p_event_data)
{
    /* The connection parameters are updated when the Discovery Procedure and
     * service configuration are complete. If the update request was successful
     * then the connection parameters resulting in lower current consumption are
     * in effect. If the request failed then more current is being consumed than
     * is necessary, but the device is still connected and working properly, so
     * just report a warning but otherwise continue to the app_state_configured
     * state.
     */
    if (p_event_data->status != sys_status_success)
    {
        DebugIfWriteString("\r\nConnection parameter update request failed on "
                           "device ");
        DebugIfWriteBdAddress(&g_app_data.devices[g_app_data.dev_num].address);
    }
    
    SetState(g_app_data.dev_num, app_state_configured);
}

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
void ReportPanic(app_panic_code panic_code)
{
    /* Raise panic */
    Panic(panic_code);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      DeviceFound
 *
 *  DESCRIPTION
 *      This function is called when a new device is discovered during the
 *      scan. It stores the information in the application data structure.
 *
 *  PARAMETERS
 *      disc_device [in]        Information for the discovered device
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void DeviceFound(DISCOVERED_DEVICE_T *disc_device)
{
    uint16 dev;                     /* Loop counter */

    /* Add device to the list of connections */
    for(dev = 0; dev < MAX_CONNECTED_DEVICES; dev++)
    {
        /* Find next unoccupied slot in the device list */
        if(!(g_app_data.devices[dev].connected) &&
            g_app_data.devices[dev].state == app_state_scanning)
        {
            break;
        }
    }

    /* No free slots found */
    if(dev == MAX_CONNECTED_DEVICES)
    {
        DebugIfWriteString("No more connections available\r\n");
        return;
    }

    /* Store the connected device number */
    g_app_data.dev_num = dev;

    /* Store the device details */
    MemCopy(&g_app_data.devices[dev].address, 
            &disc_device->address, 
            sizeof(TYPED_BD_ADDR_T));

    /* Stop scanning for advertisements */
    LsStartStopScan(FALSE, 
                    whitelist_disabled, 
                    ls_addr_type_public);

    /* Start the connection to the device */

    /* One can choose to pass the connection parameters requested by the slave 
     * (if bonded) 
     */
        
    /* In the example application the connection parameters (400ms, 400ms, 1,
     * 1000) will not be stored.
     */
    GapSetDefaultConnParams(NULL);

    /* Move to the connecting state */
    SetState(dev, app_state_connecting);

    /* Create a timer to cancel the connection request if it takes too long.
     * If the application does not leave the connecting state within
     * CONNECTING_STATE_EXPIRY_TIMER microseconds call
     * appConnectingStateTimerExpiry to issue a GATT cancel connect request.
     */
    if (g_app_data.app_timer != TIMER_INVALID)
    {
        TimerDelete(g_app_data.app_timer);
    }
    g_app_data.app_timer = TimerCreate(CONNECTING_STATE_EXPIRY_TIMER, TRUE,
                                appConnectingStateTimerExpiry);

    /* Send the connection request */
    GattConnectReq(&g_app_data.devices[dev].address, 
                   L2CAP_CONNECTION_MASTER_DIRECTED |
                   L2CAP_PEER_ADDR_TYPE_PUBLIC);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      NotifyServiceFound
 *
 *  DESCRIPTION
 *      This function is used for notifying the application about the
 *      discovered service
 *
 *  PARAMETERS
 *      pService [in]           Discovered service callback function table
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void NotifyServiceFound(SERVICE_FUNC_POINTERS_T *pService)
{
    /* Device Number */
    const uint16 dev = g_app_data.dev_num;
    /* Total number of connected services */
    uint16 *totalServices = &g_app_data.devices[dev].totalConnectedServices;

    if(*totalServices < MAX_SUPPORTED_SERV_PER_DEVICE && pService != NULL)
    {
        /* Populate the service database for the connected device */
        g_app_data.devices[g_app_data.dev_num].
                                  connected_services[*totalServices] = pService;
        
        /* Increment total number of connected services */
        (*totalServices)++;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetConnServices
 *
 *  DESCRIPTION
 *      This function returns a pointer to all the connected services
 *
 *  PARAMETERS
 *      dev [out]               Connected device number. Set to NULL if this
 *                              information is not required.
 *      totalServices [out]     Total number of connected services. Set to NULL
 *                              if this information is not required.
 *
 *  RETURNS
 *      Pointer to array of connected services' callback function tables
 *----------------------------------------------------------------------------*/
SERVICE_FUNC_POINTERS_T **GetConnServices(uint16 *dev, uint16 *totalServices)
{
    /* Device Number */
    const uint16 dev_conn = g_app_data.dev_num;
    
    if(dev != NULL)
    {
        *dev = dev_conn; /* Return the connected device number */
    }
    
    if(totalServices != NULL)
    {
        /* Return the total number of services */
        *totalServices = g_app_data.devices[dev_conn].totalConnectedServices;
    }

    return g_app_data.devices[dev_conn].connected_services;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      SetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  PARAMETERS
 *      dev [in]                Device number to change state for
 *      new_state [in]          New state to move to
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void SetState(uint16 dev, app_state new_state)
{
    /* Current state */
    const app_state old_state = g_app_data.devices[dev].state;
    
    /* Check if the new state to be set is not the same as the present state */
    if (old_state != new_state)
    {
        /* Handle exiting old state */
        switch (old_state)
        {
            case app_state_init:
            {
                appInitExit(dev);
            }
            break;

            case app_state_disconnecting:
            {
                /* Common things to do whenever the application exits the
                 * disconnecting state.
                 */

                /* This may involve freeing resources allocated to the device
                 * and reseting application data.
                 */
            }
            break;

            case app_state_connected:
            {
                /* The application may need to maintain the values of some
                 * profile specific data across connections and power cycles.
                 * These values may have changed in 'connected' state, so the
                 * application may update data stored in the NVM here.
                 */
            }
            break;

            case app_state_discovering:
            {
                /* Nothing to do */
            }
            break;

            default:
            {
                /* Nothing to do */
            }
            break;
        }

        /* Set new state */
        g_app_data.devices[dev].state = new_state;

        /* Handle entering new state */
        switch (new_state)
        {
            case app_state_scanning:
            {
                /* Start scanning for advertising devices */
                DebugIfWriteString("\r\nScanning for devices...\r\n");

                /* Reset application data */
                appDataInit();

                /* Start scanning */
                appStartScan();
            }
            break;

            case app_state_connecting:
            {
                /* This state has been introduced between app_state_connected
                 * and app_state_scanning so that if the connection does not
                 * succeed we can revert to app_state_scanning.
                 */
                DebugIfWriteString("connecting...\r\n");
            }
            break;

            case app_state_connected:
            {
                /* Common things to do whenever the application enters
                 * app_state_connected state.
                 */
#ifdef PAIRING_SUPPORT                
                /* Start a timer to trigger the Pairing Procedure after
                 * PAIRING_TIMER_VALUE ms if the remote device is using a
                 * resolvable random address and has not initiated pairing.
                 */
                StartBonding();
#endif /* PAIRING_SUPPORT */

                /* Move to the app_state_discovering state */
                appStartDiscoveryProcedure(dev);
            }
            break;

            case app_state_discovering:
            {
                /* Application enters this state from app_state_connected. In 
                 * this state the application initiates the Discover Procedure.
                 */
                DebugIfWriteString("\r\ndiscovering...\r\n");

                if (g_app_data.app_timer != TIMER_INVALID)
                {
                    TimerDelete(g_app_data.app_timer);
                }

                /* Discovery will start in DISCOVERY_START_TIMER ms */
                g_app_data.app_timer = TimerCreate(DISCOVERY_START_TIMER, TRUE,
                                            appStartDiscoveryTimerExpiry);
            }
            break;

            case app_state_configured:
            {
                /* Peer device has been configured for all the supported 
                 * services */
                DebugIfWriteString("\r\nPeer device is Configured...\r\n");
                
                NextReadWriteProcedure(TRUE);
            }
            break;

            case app_state_disconnecting:
            {
                /* Disconnect the link */
                GattDisconnectReq(g_app_data.devices[dev].connectHandle);
            }
            break;

            default:
            {
                /* Unhandled state */
                ReportPanic(app_panic_invalid_state);
            }
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetState
 *
 *  DESCRIPTION
 *      This function returns the current state of the application for the 
 *      specified device. 
 *
 *  PARAMETERS
 *      dev [in]                Device number for which to return state
 *
 *  RETURNS
 *      Current state of the specified device
 *----------------------------------------------------------------------------*/
app_state GetState(uint16 dev)
{
    return g_app_data.devices[dev].state;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      GetConnDevice
 *
 *  DESCRIPTION
 *      This function returns the connected device for which GATT procedure 
 *      is being performed. 
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Device number of currently connected device
 *----------------------------------------------------------------------------*/
uint16 GetConnDevice(void)
{
    return g_app_data.dev_num;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      DisconnectDevice
 *
 *  DESCRIPTION
 *      Disconnect the specified device.
 *
 *  PARAMETERS
 *      dev [in]                Number of device to disconnect.
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void DisconnectDevice(uint16 dev)
{
    SetState(dev, app_state_disconnecting);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      DeviceConfigured
 *
 *  DESCRIPTION
 *      Notifies the application that the specified device has been configured
 *      for all the connected services.
 *
 *  PARAMETERS
 *      dev [in]                Number of configured device
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void DeviceConfigured(uint16 dev)
{
    /* Update the connection parameters to reduce current
     * consumption.
     */
    requestConnParamUpdate(dev);
}

#ifdef PAIRING_SUPPORT
/*----------------------------------------------------------------------------*
 *  NAME
 *      StartBonding
 *
 *  DESCRIPTION
 *      Start a timer that on expiry triggers the Pairing Procedure, if the
 *      peer device has a Resolvable Random Address and has not already
 *      initiated pairing by that time.
 *
 *  PARAMETERS
 *      None
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void StartBonding(void)
{
    if (g_app_data.bonding_timer != TIMER_INVALID)
    {
        TimerDelete(g_app_data.bonding_timer);
    }
    g_app_data.bonding_timer = TimerCreate(PAIRING_TIMER_VALUE, 
                                TRUE, appPairingTimerHandlerExpiry);
}
#endif /* PAIRING_SUPPORT */

/*----------------------------------------------------------------------------*
 *  NAME
 *      NextReadWriteProcedure
 *
 *  DESCRIPTION
 *      This function initiates any read/write procedures. If argument 'next' is
 *      TRUE, it will initiate the next argument, otherwise it will initiate the 
 *      procedure for the current characteristic.
 *
 *  PARAMETERS
 *      next [in]               Whether to initiate the next argument
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void NextReadWriteProcedure(bool next)
{
    /* Read all the characteristics of the Device Info Service, if the service
     * is present on the peer device.
     */
    /* Pointer to Device Info Service callback function table */
    static SERVICE_FUNC_POINTERS_T *pService = NULL;
    /* Current characteristic being read */
    static uint16 char_type = dev_info_manufacture_name;

    /* The first time this function is called, initialise pService to point to
     * the Device Information Service callback function table.
     */
    if(pService == NULL)
    {
        const uint16 uuid = UUID_DEVICE_INFO_SERVICE;

        pService = GattFindServiceByUuid(GATT_UUID16, &uuid);
    }

    if(!next && char_type > dev_info_manufacture_name)
    {
        /* If we're not moving onto the next characteristic, reset char_type
         * so that the last read characteristic is re-read.
         */
        char_type--;
    }

    /* Read the next supported characteristic */
    while(char_type < dev_info_type_invalid)
    {
        if(GattReadRequest(g_app_data.dev_num, pService, char_type))
        {
            /* If the current characteristic is supported, exit the loop and
             * wait for the GATT_READ_CHAR_VAL_CFM event.
             */
            break;
        }
        
        /* If the current characteristic is not supported, try the next one */
        char_type++;
    }

    /* If all the characteristics have been read */
    if(char_type == dev_info_type_invalid)
    {
        uint16 dev;                 /* Loop counter */

        /* Reset char_type ready for the next connection */
        char_type = dev_info_manufacture_name;

        /* The connected device is configured. Start scanning for the next
         * device if there is room in the application data structure.
         */
        /* Find the device number of the next spare slot in the application
         * data structure.
         */
        for (dev=0; dev < MAX_CONNECTED_DEVICES; dev++)
        {
            if(!g_app_data.devices[dev].connected &&
               g_app_data.devices[dev].state == app_state_init)
            {
                /* Spare slot found, exit the loop */
                break;
            }
        }
        
        if(dev == MAX_CONNECTED_DEVICES)
        {
            /* Maximum limit reached. No more devices may be connected. */
            return;
        }

        /* Start scanning for the next device */
        SetState(dev, app_state_scanning);

        return;
    }

    /* Increment char_type, so that next time a read request for the next
     * characteristic will be sent.
     */
    char_type++;
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
void AppPowerOnReset(void)
{
    /* Code that is only executed after a power-on reset or firmware panic
     * should be implemented here - e.g. configuring application constants
     */
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
 *      after AppPowerOnReset().
 *
 *  PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state last_sleep_state)
{
    uint16 dev;                     /* Loop counter */

    /* Initialise application debug */
    DebugIfInit();
    
    /* Announce application on the UART */
    DebugIfWriteString("\r\n\r\n***********************\r\n");
    DebugIfWriteString("GATT client GAP central\r\n\r\n");

    /* Initialise the application timers */
    TimerInit(MAX_APP_TIMERS, (void*)app_timers);
    g_app_data.app_timer = TIMER_INVALID;
#ifdef PAIRING_SUPPORT
    g_app_data.bonding_timer = TIMER_INVALID;
#endif /* PAIRING_SUPPORT */

    /* Initialise the GATT Client application state for each device */
    for(dev = 0; dev < MAX_CONNECTED_DEVICES; dev ++)
    {
        g_app_data.devices[dev].state = app_state_init;
    }
    
    /* Initialise the connected device numbers */
    g_app_data.dev_num = MAX_CONNECTED_DEVICES;

    /* Initialise the bonded device numbers */
    g_app_data.nvm_dev_num = MAX_BONDED_DEVICES;

    /* Initialise GATT entity */
    GattInit();

    /* Install mandatory GATT Client functionality. This function must be
     * called after GattInit and before any other GATT Client functions from
     * the firmware API.
     */
    GattInstallClientRole();

#ifdef NVM_TYPE_EEPROM
    /* Configure the NVM manager to use I2C EEPROM for NVM store */
    NvmConfigureI2cEeprom();
#elif NVM_TYPE_FLASH
    /* Configure the NVM Manager to use SPI flash for NVM store. */
    NvmConfigureSpiFlash();
#endif /* NVM_TYPE_EEPROM */

    Nvm_Disable();

    /* Read persistent storage */
    readPersistentStore(g_app_data.dev_num, g_app_data.nvm_dev_num);

    /* Tell Security Manager module what value it needs to initialise its
     * diversifier to.
     */
    SMInit(0);

    /* Initialise number of connections */
    g_app_data.num_conn = 0;

    /* Start scanning for advertisements from the first device (0) */
    SetState(0, app_state_scanning);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcesSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  PARAMETERS
 *      id   [in]               System event ID
 *      data [in]               Event data
 *
 *  RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{
    /* This example application does not process any system events */
}

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
 *      p_event_data [in]       LM event data
 *
 *  RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
 *----------------------------------------------------------------------------*/
bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *p_event_data)
{
    switch (event_code)
    {

        /* Handle events received from Firmware */

        case LM_EV_CONNECTION_COMPLETE:
        {
            /* Handle the LM connection complete event. */
            handleSignalLmEvConnectionComplete(
                          &((LM_EV_CONNECTION_COMPLETE_T*)p_event_data)->data);
        }
        break;

        case GATT_CONNECT_CFM:
        {
            /* Confirmation for the completion of GattConnectReq() procedure */
            handleSignalGattConnectCfm((GATT_CONNECT_CFM_T*)p_event_data);
        }
        break;

        case SM_KEY_REQUEST_IND:
        {
            /* Indicates that the Security Manager cannot find 
             * security keys for the host in its persistent store. 
             * Application responds with either a SM_KEYSET_T or 
             * NULL pointer in SMKeyRequestResponse()
             */
            handleSignalSmKeyRequestInd((SM_KEY_REQUEST_IND_T *)p_event_data);
        }
        break;

        case SM_KEYS_IND:
        {
            /* Indication for the keys and associated security information
             * on a connection that has completed Short Term Key Generation 
             * or Transport Specific Key Distribution
             */
            handleSignalSmKeysInd((SM_KEYS_IND_T *)p_event_data);
        }
        break;

        case SM_SIMPLE_PAIRING_COMPLETE_IND:
        {
            /* Indication for completion of Pairing procedure */
            handleSignalSmSimplePairingCompleteInd(
                (SM_SIMPLE_PAIRING_COMPLETE_IND_T *)p_event_data);
        }
        break;

        case LM_EV_ENCRYPTION_CHANGE:
        {
            /* Indication for encryption change event */

            /* Nothing to do */
        }
        break;

        case GATT_DISCONNECT_IND:
        {
            /* Disconnect procedure triggered by remote host or due to 
             * link loss is considered complete on reception of 
             * LM_EV_DISCONNECT_COMPLETE event. So, it gets handled on 
             * reception of LM_EV_DISCONNECT_COMPLETE event.
             */
        }
        break;

        case GATT_DISCONNECT_CFM:
        {
            /* Confirmation for the completion of GattDisconnectReq()
             * procedure is ignored as the procedure is considered complete 
             * on reception of LM_EV_DISCONNECT_COMPLETE event. So, it gets 
             * handled on reception of LM_EV_DISCONNECT_COMPLETE event.
             */
        }
        break;

        case LM_EV_DISCONNECT_COMPLETE:
        {
            /* Disconnect procedures either triggered by application or remote
             * host or link loss case are considered completed on reception 
             * of LM_EV_DISCONNECT_COMPLETE event
             */
             handleSignalLmDisconnectComplete(
                    &((LM_EV_DISCONNECT_COMPLETE_T *)p_event_data)->data);
        }
        break;

        case GATT_CANCEL_CONNECT_CFM:
        {
            /* Confirmation for the completion of GattCancelConnectReq()
             * procedure 
             */
            handleSignalGattCancelConnectCfm();
        }
        break;

        case LS_CONNECTION_UPDATE_SIGNALLING_IND:
        {
            /* This event is raised on a master after a slave initiates a 
             * Connection Parameter Update procedure. It is handled by a call 
             * to LsConnectionUpdateSignalingRsp() accepting or rejecting 
             * the proposed connection parameters
             */
            
            /* Vendor may choose to accept/reject the connection paramters 
             * received in this call, if the connection parameters are not 
             * required to be stored in the application and the application 
             * always accepts the params, then this call is not required
             */
        }
        break;

        case LS_CONNECTION_PARAM_UPDATE_CFM:
        {
            /* This event is raised after a Connection Parameter Update
             * procedure initiated by the master has completed.
             */
            handleSignalLsConnectionParamUpdateCfm(
                              (LS_CONNECTION_PARAM_UPDATE_CFM_T *)p_event_data);
        }
        break;

        default:
        {
            /* All the Discovery Procedure events are handled here */
            GattDiscoveryEvent(event_code, p_event_data);
        }
        break;

    }

    return TRUE;
}
