/***************************************************************************//**
 * @file main.c
 * @brief main.c
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
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
  #include "sl_power_manager.h"
#endif
#include "app_init.h"
#include "app_process.h"
#if defined(SL_CATALOG_KERNEL_PRESENT)
  #include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
  #include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sl_led.h"
#include "sl_simple_led_instances.h"
#include "sl_sleeptimer.h"

#include "string.h"
#include "strings.h"
#include "stdio.h"

#include "sl_led.h"
#include "sl_simple_led_instances.h"
#include "sl_button.h"
#include "sl_simple_button_instances.h"

#include "sl_uartdrv_usart_vcom_config.h"
#include "sl_uartdrv_instances.h"
#include "sl_board_control_config.h"
#include "em_emu.h"
#include "sl_power_manager_config.h"
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define QUEUE_DEFAULT_LENGTH 4
#define RETRANSMISSION_BUFFER_DEFAULT_LENGTH 4
#define RECEIVE_WINDOW_MICROSECONDS 1000000
#define RECEIVE_WINDOW_MILLISECONDS RECEIVE_WINDOW_MICROSECONDS/1000

enum wupSequence{
  Wa,
  Wb,
  Wr
};

#pragma pack(push,1)
typedef struct
{
  uint16_t wupSeq; //Wa, Wb, Wr
  uint16_t pktSeq; //Packet Sequence #
} pkt_header_t;

typedef struct
{
  pkt_header_t header;
  uint8_t payload[12];
} pkt_t;
#pragma pack(pop)
// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
///Transmitter Task
static StaticTask_t transmitterTaskTCB;
static StackType_t transmitterTaskStack[configMINIMAL_STACK_SIZE];
static void transmitterTaskFunction (void*);
static TaskHandle_t transmitterTaskHandle;

///Receiver Task
static StaticTask_t receiverTaskTCB;
static StackType_t receiverTaskStack[configMINIMAL_STACK_SIZE];
static void receiverTaskFunction (void*);
static TaskHandle_t receiverTaskHandle;

///Delayer Task
static StaticTask_t delayerTaskTCB;
static StackType_t delayerTaskStack[configMINIMAL_STACK_SIZE];
static void delayerTaskFunction (void*);
static TaskHandle_t delayerTaskHandle;

///Debug Task
static StaticTask_t debugTaskTCB;
static StackType_t debugTaskStack[configMINIMAL_STACK_SIZE];
static void debugTaskFunction (void*);
static TaskHandle_t debugTaskHandle;

///Idle task and Timer task definitions
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

/// Queue handle and space
static QueueHandle_t transmitterQueueHandle;
static StaticQueue_t transmitterQueueDataStruct;
static uint8_t transmitterQueue[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];

///RFSense callback
static void rfSenseCb(void);

///Task initiation grouping
static void initTasks(void);

///Callback Function
static void timerCallback(sl_sleeptimer_timer_handle_t *handle, void *data);
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
static RAIL_Handle_t rail_handle;

/// RAIL tx and rx queue
static uint8_t railRxFifo[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];
static uint16_t rxFifoSize = sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH;
static uint8_t railTxFifo[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];

///Retransmission buffer queue
static pkt_t retransmissionBuffer[sizeof(pkt_t) * RETRANSMISSION_BUFFER_DEFAULT_LENGTH];
static uint32_t retransmissionBufferIndex = 0;

/// Dummy struct to use to generate received and transmitted packets
static pkt_t rxPacket, retransmissionPacket;

///Rx Packet handle, details and info
static RAIL_RxPacketHandle_t packet_handle;
static RAIL_RxPacketInfo_t packet_info;

/// Pointer used to force context switch from ISR
static BaseType_t xHigherPriorityTaskWoken;

///Packet sequence number and WUP Sequence number
static uint32_t lastRxPktSequenceNumber = -1;
static uint16_t wupSeq = Wa;

///VCOM Serial print buffer
static uint8_t buffer[100];

///For counter
static int retransmissionCnt, shiftCnt = 0;

static sl_sleeptimer_timer_handle_t delayerSleeptimerHandle;
static volatile bool wait;
// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Main function
 *****************************************************************************/
int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  rail_handle = app_init();

  //Initialize the tasks
  initTasks();

  //Init Queues
  transmitterQueueHandle = xQueueCreateStatic(QUEUE_DEFAULT_LENGTH, sizeof(pkt_t), transmitterQueue, &transmitterQueueDataStruct);

  // setting tx fifo (rx fifo set during radio init)
  RAIL_SetTxFifo (rail_handle, railTxFifo, 0,sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH);

  //enabling vcom
  GPIO_PinOutSet (SL_BOARD_ENABLE_VCOM_PORT, SL_BOARD_ENABLE_VCOM_PIN);

#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action(rail_handle);

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
///Idle and Timer Task definition
void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer,
                               StackType_t **ppxIdleTaskStackBuffer,
                               uint32_t *pulIdleTaskStackSize)
{
  //Declare static variables so they're not placed on the stack

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer,
                                StackType_t **ppxTimerTaskStackBuffer,
                                uint32_t *pulTimerTaskStackSize)
{

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

///Receiver Task
void receiverTaskFunction (void *rt){
  while (1)
    {

      ulTaskNotifyTake (pdFALSE, portMAX_DELAY);
      sl_sleeptimer_restart_timer_ms(&delayerSleeptimerHandle, 1000, timerCallback, NULL, 0, 0);

      packet_handle = RAIL_GetRxPacketInfo (rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);

      if (packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
        {
          RAIL_CopyRxPacket (&rxPacket, &packet_info);
          RAIL_ReleaseRxPacket (rail_handle, packet_handle);
          if(rxPacket.header.wupSeq == Wr){
              //Wait 100ms to transmit
              sl_sleeptimer_delay_millisecond(100);
              //CHECK THE RETRANSMISSION BUFFER AND RETRANSMIT THE LOST PACKET
              for(retransmissionCnt = 0; retransmissionCnt < RETRANSMISSION_BUFFER_DEFAULT_LENGTH; retransmissionCnt++){
                  if(retransmissionBuffer[retransmissionCnt].header.pktSeq == rxPacket.header.pktSeq){
                      xQueueSend(transmitterQueueHandle, &retransmissionBuffer[retransmissionCnt], 0);
                  }
              }
          }else if(rxPacket.header.wupSeq == wupSeq && rxPacket.header.pktSeq == lastRxPktSequenceNumber + 1){
              snprintf (buffer, 100, "Received a flood packet:\r\nPacket Sequence number: %lu\r\nWUP sequence: %lu\r\n", rxPacket.header.pktSeq, rxPacket.header.wupSeq);

              Ecode_t return_code;
              do
                {
                  return_code = UARTDRV_TransmitB (sl_uartdrv_usart_vcom_handle, &buffer[0], strlen(buffer));
                }
              while (ECODE_OK != return_code);

              //Retransmit packet in the WSN
              //xQueueSend(transmitterQueueHandle, &rxPacket, 0);

              //Update the last packet received and WUP sequence
              lastRxPktSequenceNumber = rxPacket.header.pktSeq;
              if(wupSeq == Wa){
                  wupSeq = Wb;
              } else{
                  wupSeq = Wa;
              }

              //Add the packet to the retransmission buffer
              if(retransmissionBufferIndex < RETRANSMISSION_BUFFER_DEFAULT_LENGTH){
                  retransmissionBuffer[retransmissionBufferIndex] = rxPacket;
                  retransmissionBufferIndex++;
              }else{
                  //SHIFT BUFFER RIGHT 1 PLACE AND ADD NEW PACKET AT THE END
                  for(shiftCnt = 0; shiftCnt < RETRANSMISSION_BUFFER_DEFAULT_LENGTH - 1; shiftCnt++){
                      retransmissionBuffer[shiftCnt] = retransmissionBuffer[shiftCnt + 1];
                  }
                  retransmissionBuffer[shiftCnt] = rxPacket;
              }
          }else{
              //WE LOST A PACKET SOMEWHERE WE NEED TO ASK OUR NEIGHBOURS TO RETRANSMIT THE LOST PACKET
              retransmissionPacket.header.wupSeq = Wr;
              retransmissionPacket.header.pktSeq = lastRxPktSequenceNumber + 1;
              snprintf(&(retransmissionPacket.payload[0]), 12, "Rtx Seq: %lu", lastRxPktSequenceNumber + 1);

              xQueueSend(transmitterQueueHandle, &retransmissionPacket, 0);
          }
        }

    }
}

///Transmitter task
void transmitterTaskFunction (void *tt)
{
  while (1)
    {
      //CREATE A QUEUE TO RECEIVE PACKETS TO SEND
      xQueueReceive(transmitterQueueHandle, &retransmissionPacket, portMAX_DELAY);

      //Simulate sending a WUP packet to wake up nodes on the sub GHZ frequency.
      //In our case we send the actual packet
      RAIL_WriteTxFifo (rail_handle, (uint8_t*) &retransmissionPacket, sizeof(pkt_t), false);
      RAIL_StartTx (rail_handle, 21, 0, NULL);
      //Wait for 100ms to be sure that the node have woken up
      //We are still in the rx wake up window (1sec)
      sl_sleeptimer_delay_millisecond (100);
      //Send the actual flood data packet
      RAIL_WriteTxFifo (rail_handle, (uint8_t*) &retransmissionPacket, sizeof(pkt_t), false);
      while (RAIL_STATUS_NO_ERROR != RAIL_StartTx (rail_handle, 0, 0, NULL));
    }
}

void timerCallback(sl_sleeptimer_timer_handle_t *handle, void *data){
  wait = false;
}

///Delayer task, avoids we immediately go to sleep after waking up with RFSense
void delayerTaskFunction (void *dt)
{
  while (1)
    {
      ulTaskNotifyTake (pdTRUE, portMAX_DELAY);
      //delay just to avoid falling into idle task
      //sl_sleeptimer_delay_millisecond(RECEIVE_WINDOW_MILLISECONDS);
      wait = true;
      sl_sleeptimer_start_timer_ms(&delayerSleeptimerHandle, 1000, timerCallback, NULL, 0, 0);

      while(wait);

      __asm__("nop");
    }
}

///DEBUG TASK
void debugTaskFunction (void *dt)
{
  while (1)
    {
      ulTaskNotifyTake (pdTRUE, portMAX_DELAY);

      for(int i = 0; i < retransmissionBufferIndex; i++){
          snprintf (buffer, 100, "\r\nPacket buffer position: %u\r\nPacket Sequence number: %lu\r\nWUP sequence: %lu\r\n", i, retransmissionBuffer[i].header.pktSeq, retransmissionBuffer[i].header.wupSeq);

          Ecode_t return_code;
          do
            {
              return_code = UARTDRV_TransmitB (sl_uartdrv_usart_vcom_handle, &buffer[0], strlen(buffer));
            }
          while (ECODE_OK != return_code);
      }
    }
}


///Initialize all the tasks
void initTasks(){

   //Receiver Task
   receiverTaskHandle = xTaskCreateStatic (receiverTaskFunction, "receiverTask", configMINIMAL_STACK_SIZE, NULL, 2, receiverTaskStack, &receiverTaskTCB);
    if (receiverTaskHandle == NULL)
      {
        exit (0);
      }

    //Transmitter Task
    transmitterTaskHandle = xTaskCreateStatic (transmitterTaskFunction, "transmitterTask", configMINIMAL_STACK_SIZE, NULL, 3, transmitterTaskStack, &transmitterTaskTCB);
    if (transmitterTaskHandle == NULL)
      {
        exit (0);
      }

    //Delayer Task
    //This task prevents going into sleep mode again after waking up. "Sleep mode" is just the Idle Task running,
    //when we wake up with RFSense we would immediately fall back into it after the callback. By notifying this task we prevent this.
    delayerTaskHandle = xTaskCreateStatic (delayerTaskFunction, "delayerTask", configMINIMAL_STACK_SIZE, NULL, 1, delayerTaskStack, &delayerTaskTCB);
    if (delayerTaskHandle == NULL)
      {
        exit (0);
      }

    //debug Task
    debugTaskHandle = xTaskCreateStatic (debugTaskFunction, "debugTask", configMINIMAL_STACK_SIZE, NULL, 2, debugTaskStack, &debugTaskTCB);
     if (debugTaskHandle == NULL)
       {
         exit (0);
       }
}

/**
 * Function that customize setting rx fifo
 * @param railHandle
 * @return
 */
RAIL_Status_t RAILCb_SetupRxFifo (RAIL_Handle_t railHandle)
{
  RAIL_Status_t status = RAIL_SetRxFifo (railHandle, &railRxFifo[0], &rxFifoSize);
  if (rxFifoSize != sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH)
    {
      // We set up an incorrect FIFO size
      return RAIL_STATUS_INVALID_PARAMETER;
    }
  if (status == RAIL_STATUS_INVALID_STATE)
    {
      // Allow failures due to multiprotocol
      return RAIL_STATUS_NO_ERROR;
    }
  return status;
}

///RAIL event handler
void
sl_rail_app_on_event (RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_CAL_NEEDED)
    {
      RAIL_Calibrate (rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    }
  if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
    {
      //new rx -> deferred handler architecture
      RAIL_HoldRxPacket (rail_handle);
      vTaskNotifyGiveFromISR (receiverTaskHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  if (events & RAIL_EVENTS_TX_COMPLETION)
    {
      sl_led_turn_off(&sl_led_led0);
      sl_led_turn_on(&sl_led_led0);
      sl_udelay_wait(100000);
      sl_led_turn_off(&sl_led_led0);
    }
}

///RFSense Callback function
void rfSenseCb ()
{
  //We've woken up with RFSense, now we schedule a receiving window of 1s
  //and notify the delayer task so we don't immediately go to sleep
  RAIL_Status_t status = RAIL_StartRx (rail_handle, 0 , NULL);
  if (status == RAIL_STATUS_NO_ERROR)
    {
      xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR (delayerTaskHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

///Idle Task Hook, we turn off the radio and start the RFSense peripheral on the Sub GHZ freq before entering "sleep mode"
void vApplicationIdleHook ()
{
  // Starting RFSENSE before going to sleep
  RAIL_Idle (rail_handle, RAIL_IDLE, true);
  RAIL_StartRfSense (rail_handle, RAIL_RFSENSE_SUBGHZ, 50, rfSenseCb);
}


///BUTTON INTERRUPT HANDLER
void sl_button_on_change(const sl_button_t *handle)
{
  //BTN0 PRESSED
  if(handle == &sl_button_btn0){ //check if the memory address pointed by handle is the same as the btn0 variable
    if(sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED){ //event is only for when the button is pressed and not when it's released
        //Use a task notification to unblock the transmitter task
        xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(debugTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

