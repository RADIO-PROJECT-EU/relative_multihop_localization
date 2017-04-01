/*******************************************************************************
 *  Copyright (c) 2013 - 2016 Qualcomm Technologies International, Ltd.
 *  Part of CSR uEnergy SDK 2.6.0
 *  Application version 2.6.0.0
 *
 * FILE
 *      user_config.h
 *
 * DESCRIPTION
 *      This file contains definitions which will enable customisation of the
 *      application.
 *
 ******************************************************************************/

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

#define USE_STATIC_RANDOM_ADDRESS
#undef USE_STATIC_RANDOM_ADDRESS

/* This macro when defined enables the debug output on UART */
#define DEBUG_OUTPUT_ENABLED


/* The FILTER_DEVICE_BY_SERVICE macro indicates that the Client should not
 * connect to any Servers unless they are advertising at least one of the
 * services supported by the Client (defined by g_supported_services in
 * gatt_client.c).
 */
#define FILTER_DEVICE_BY_SERVICE


/* Service related macros */

/* The MAX_SUPPORTED_SERVICES macro defines the number of services supported by
 * the Client application. This may be >= MAX_SUPPORTED_SERV_PER_DEVICE as it
 * may not be mandatory for a Server to provide all the supported services.
 */
#define MAX_SUPPORTED_SERVICES                    (10)

/* The MAX_SUPPORTED_SERV_PER_DEVICE macro defines the maximum number of
 * supported services that may be provided by each Server. This will be <=
 * MAX_SUPPORTED_SERVICES  as it may not be mandatory for a Server to provide
 * all the supported services.
 */
#define MAX_SUPPORTED_SERV_PER_DEVICE             (5)

/* The MAX_CONNECTED_DEVICES macro defines the number of devices that can be
 * connected at any given time to the Client device. It must not exceed 1.
 */
#define MAX_CONNECTED_DEVICES                     (1)

/* The MAX_BONDED_DEVICES macro defines the number of devices whose information
 * can be stored in the NVM. It should not exceed 3.
 */
#define MAX_BONDED_DEVICES                        (1)

#endif /* __USER_CONFIG_H__ */
