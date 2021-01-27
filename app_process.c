/***************************************************************************//**
 * @file app_process.c
 * @brief app_process.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "rail.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(RAIL_Handle_t rail_handle)
{
  (void) rail_handle;

  ///////////////////////////////////////////////////////////////////////////
  // Put your application code here!                                       //
  // This is called infinitely.                                            //
  // Do not call blocking functions from here!                             //
  ///////////////////////////////////////////////////////////////////////////
}

///******************************************************************************
// * RAIL callback, called if a RAIL event occurs
// *****************************************************************************/
//void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
//{
//  (void) rail_handle;
//  (void) events;
//
//  ///////////////////////////////////////////////////////////////////////////
//  // Put your RAIL event handling here!                                    //
//  // This is called from ISR context.                                      //
//  // Do not call blocking functions from here!                             //
//  ///////////////////////////////////////////////////////////////////////////
//}


// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
