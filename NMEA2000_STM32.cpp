/*
 NMEA2000_STM32.cpp


 Inherited NMEA2000 object for the STM32F105 internal CAN
 See also NMEA2000 library.

 Only use with STM32 HAL for your MCU!


 Copyright (c) 2022 Minos Eigenheer

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 ACTION OF
 CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE
 OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "NMEA2000_STM32.hpp"

#define NMEA2000_STM32_DEBUG
#define NMEA2000_STM32_DEBUG_ERRORS

#if defined(NMEA2000_STM32_DEBUG)
# define DbgPrintfSWD(fmt, args...)     printf(fmt, ## args); printf("\n")
#else
# define DbgPrintfSWD(fmt, args...)
#endif

#if defined(NMEA2000_STM32_DEBUG_ERRORS)
# define DbgPrintfSWDErr(fmt, args...)     printf("#red#"); printf(fmt, ## args); printf("\n")
#else
# define DbgPrintfSWDErr(fmt, args...)
#endif
//*****************************************************************************
tNMEA2000_STM32::tNMEA2000_STM32(CAN_HandleTypeDef *_canBus, CANbaudRatePrescaler _CANbaudRate) :
#if defined(_NMEA2000_H_)
		tNMEA2000(),
#endif
		canBus(_canBus),
		CANbaudRate(_CANbaudRate) {

	//NMEA2000_STM32_instance = this;
	canInstances.push_back(this); // add this instance to canInstances

	rxRing = 0;
	txRing = 0;
}

//*****************************************************************************
bool tNMEA2000_STM32::CANOpen() {

	bool ret = true;

	// CAN initialisation instead of using the by the STM32cubeIDE configuration tool generated init function
	if (CANInit() != HAL_OK) {
		ret = false;
	}

	// activate CAN callback interrupts
	if (HAL_CAN_ActivateNotification(canBus,
			CAN_IT_RX_FIFO0_MSG_PENDING |
			CAN_IT_TX_MAILBOX_EMPTY |
			CAN_IT_ERROR |
			CAN_IT_ERROR_WARNING
		) != HAL_OK) {
		ret = false;
	}

	// Enable CAN
	if (HAL_CAN_Start(canBus) != HAL_OK) {
		ret = false;
	}

	DbgPrintfSWD("CAN started");
	return ret;
}

//*****************************************************************************
bool tNMEA2000_STM32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent) {
	//TODO wait_sent
	uint8_t prio = (uint8_t)((id >> 26) & 0x7);

	//TODO use lowest CAN ID priority in the TX mailboxes
	//Problem is I can't chose mailbox with HAL CAN driver
	//Use mailboxes in FIFO mode for now
	//uint32_t mailbox;
	//if      (prio <= 2) {mailbox = CAN_TX_MAILBOX0;}
	//else if (prio <= 4) {mailbox = CAN_TX_MAILBOX1;}
	//else                {mailbox = CAN_TX_MAILBOX2;}
	//bool TxMessagePending = HAL_CAN_IsTxMessagePending(canBus, mailbox);

	bool ret = false;

	HAL_CAN_DeactivateNotification(canBus, CAN_IT_TX_MAILBOX_EMPTY);

	bool TxMailboxesFull = HAL_CAN_GetTxMailboxesFreeLevel(canBus) == 0;
	bool SendFromBuffer = false;

	// If TX buffer has already some frames waiting with higher prio or mailbox is full, buffer frame
	if ( !txRing->isEmpty(prio) || TxMailboxesFull ) {
		CAN_message_t *msg = txRing->getAddRef(prio);
		if ( msg!=0 ) {
			msg->id = id;
			msg->flags.extended = 1;
			if ( len > 8 ) len = 8;
			msg->len = len;
			memcpy(msg->buf, buf, len);
			ret = true;
			//frame buffered
			DbgPrintfSWD("frame buffered");
		}
		SendFromBuffer = true;
	}

	if ( !TxMailboxesFull ) {
		if ( SendFromBuffer ) {
			ret = SendFromTxRing();
		} else {
			ret = CANWriteTxMailbox(id, len, buf, 1);
		}
		/* transmit entry accepted */
	}

	HAL_CAN_ActivateNotification(canBus, CAN_IT_TX_MAILBOX_EMPTY);

	return ret;
}

//*****************************************************************************
bool tNMEA2000_STM32::CANSendFrameStruct(tNMEA2000_STM32::CAN_message_t* message) {
	return CANSendFrame(message->id, message->len, message->buf, message->flags.extended);
}
//*****************************************************************************
bool tNMEA2000_STM32::CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) {

	bool ret = false;

	HAL_CAN_DeactivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	const CAN_message_t *msg = rxRing->getReadRef();
	if ( msg!=0 ) {
	    id = msg->id;
	    len = msg->len;
		if ( len > 8 ) len = 8;
	    memcpy(buf, msg->buf, len);
	    ret = true;
	}

	HAL_CAN_ActivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	return ret;

}
//*****************************************************************************
bool tNMEA2000_STM32::CANGetFrameStruct(tNMEA2000_STM32::CAN_message_t* message) {

	bool ret = false;

	HAL_CAN_DeactivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	const CAN_message_t *msg = rxRing->getReadRef();
	if ( msg!=0 ) {
	    //message = msg;
	    memcpy(message, msg, sizeof(*msg));
	    ret = true;
	}

	HAL_CAN_ActivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	return ret;

}

// *****************************************************************************
void tNMEA2000_STM32::InitCANFrameBuffers() {

  if ( MaxCANReceiveFrames == 0 ) MaxCANReceiveFrames = 32; // Use default, if not set
  if ( MaxCANReceiveFrames < 10 ) MaxCANReceiveFrames = 10; // Do not allow less than 10 - should have enough memory.
  if ( MaxCANSendFrames == 0 ) MaxCANSendFrames = 50;  // Use big enough default buffer
  if ( MaxCANSendFrames < 30 ) MaxCANSendFrames = 30; // Do not allow less than 30 - should have enough memory.

  //TODO deleting ring buffer results in hard fault!
  // if buffer is initialized with a different size delete it
  if ( rxRing != 0 && rxRing->getSize() != MaxCANReceiveFrames ) {
    delete rxRing;
    rxRing = 0;
  }
  if ( txRing != 0 && txRing->getSize() != MaxCANSendFrames ) {
    delete txRing;
    txRing = 0;
  }
  //							tPriorityRingBuffer(uint16_t _size, uint8_t _maxPriorities=1);
  if ( rxRing == 0 ) rxRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANReceiveFrames, 7);
  if ( txRing == 0 ) txRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANSendFrames, 7);

  DbgPrintfSWD("frame buffer initaialized");

}

// *****************************************************************************
bool tNMEA2000_STM32::CANWriteTxMailbox(unsigned long id, unsigned char len, const unsigned char *buf, bool extended) {

	if (extended) {
		CANTxHeader.IDE = CAN_ID_EXT;
		CANTxHeader.ExtId = id;
	} else {
		CANTxHeader.IDE = CAN_ID_STD;
		CANTxHeader.StdId = id;
	}
	CANTxHeader.RTR = CAN_RTR_DATA;
	CANTxHeader.DLC = len;
	CANTxHeader.TransmitGlobalTime = DISABLE;

    if ( len > 8 ) len = 8;
    memcpy(CANTxdata, buf, len);
	//for (int i = 0; i < len; i++) {
	//	CANTxdata[i] = buf[i];
	//}

	// send message
	if (HAL_CAN_AddTxMessage(canBus, &CANTxHeader, CANTxdata, &CANTxMailbox) == HAL_OK) {
		DbgPrintfSWD("Added frame to TX mailbox %lx", id);
		return true;
	} else {
		return false;
	}

}

// *****************************************************************************
bool tNMEA2000_STM32::SendFromTxRing() {
	const CAN_message_t *txMsg;
	txMsg = txRing->getReadRef(); // always get highest prio message from the buffer
	if ( txMsg != 0 ) {
		return CANWriteTxMailbox(txMsg->id, txMsg->len, txMsg->buf, txMsg->flags.extended);
	} else {
		return false;
	}

}

// *****************************************************************************
void tNMEA2000_STM32::CANReadRxMailbox(CAN_HandleTypeDef *hcan, uint32_t CANRxFIFO) {
	CAN_message_t *rxMsg;
	uint8_t prio;

	if (hcan == canBus) {
		if (HAL_CAN_GetRxMessage(hcan, CANRxFIFO, &CANRxHeader, CANRxdata) == HAL_OK) {
			prio = (uint8_t)((CANRxHeader.ExtId >> 26) & 0x7);
			rxMsg = rxRing->getAddRef(prio);
			if ( rxMsg!=0 ) {
				rxMsg->len = CANRxHeader.DLC;
				if ( rxMsg->len > 8 ) rxMsg->len = 8;
				rxMsg->flags.remote = CANRxHeader.RTR == CAN_RTR_REMOTE;
				rxMsg->flags.extended = CANRxHeader.IDE == CAN_ID_EXT;
				rxMsg->id = CANRxHeader.ExtId;
			}
			memcpy(rxMsg->buf, CANRxdata, rxMsg->len);
			DbgPrintfSWD("Got CAN message %lx", rxMsg->id);
		}
	}

	// I think we don't have to check the fifo fill level if we use interrups?
	// HAL_CAN_GetRxFifoFillLevel(*canBus, CAN_RX_FIFO1);

}


/**
  * @brief CAN Initialization Function
  * @retval bool success or not
  */
HAL_StatusTypeDef tNMEA2000_STM32::CANInit()
{
	// CAN1000kbitPrescaler, TimeSeg1 and TimeSeg2 are configured for 1000 kbit/s @ defined clock speed
	// Baud rate has to be dividable by 1000 (500, 250, 200, 125, 100...)

	CAN_TypeDef *CANinstance;
#ifdef CAN1
	if (canBus == &hcan1) {
		CANinstance = CAN1;
		DbgPrintfSWD("Init CAN 1");
	}
#ifdef CAN2
	else if (canBus == &hcan2) {
		CANinstance = CAN2;
		DbgPrintfSWD("Init CAN 2");
	}
#endif
#ifdef CAN3
	else if (canBus == &hcan3) {
		CANinstance = CAN3;
		DbgPrintfSWD("Init CAN 3");
	}
#endif
	else {
		// CAN_HandleTypeDef *hcan is unknown
		return HAL_ERROR;
	}
#endif

	uint32_t CAN1000kbitPrescaler;
	uint32_t CANtimeSeg1;
	uint32_t CANtimeSeg2;


	// usually the APB1 clock is running at the following speed if max clock frequencies are used:
	// STM32F103/105/107   36'000'000 Hz
	// STM32F405/407       42 000'000 Hz
	uint32_t APB1clockSpeed = HAL_RCC_GetPCLK1Freq();

	if (APB1clockSpeed == 24000000) {
		CAN1000kbitPrescaler = 2;
		CANtimeSeg1 = CAN_BS1_10TQ;
		CANtimeSeg2 = CAN_BS2_1TQ;
	}
	else if (APB1clockSpeed == 36000000) {
		CAN1000kbitPrescaler = 2;
		CANtimeSeg1 = CAN_BS1_15TQ;
		CANtimeSeg2 = CAN_BS2_2TQ;
	}
	else if (APB1clockSpeed == 42000000) {
		CAN1000kbitPrescaler = 3;
		CANtimeSeg1 = CAN_BS1_11TQ;
		CANtimeSeg2 = CAN_BS2_2TQ;
	}
	else if (APB1clockSpeed == 48000000) {
		CAN1000kbitPrescaler = 3;
		CANtimeSeg1 = CAN_BS1_13TQ;
		CANtimeSeg2 = CAN_BS2_2TQ;
	}
	else {
		// There are no settings four your ABT1 clock speed yet!
		// On the following website you can find a matching prescaler, TS1 and TS2
		// Add the values for your clock speed and 1000kbit/s
		// http://www.bittiming.can-wiki.info/?CLK=36&ctype=bxCAN&SamplePoint=87.5
		return HAL_ERROR;
	}

	canBus->Instance = CANinstance;
	canBus->Init.Prescaler = (CAN1000kbitPrescaler * CANbaudRate);
	canBus->Init.Mode = CAN_MODE_NORMAL;
	canBus->Init.SyncJumpWidth = CAN_SJW_1TQ;
	canBus->Init.TimeSeg1 = CANtimeSeg1;
	canBus->Init.TimeSeg2 = CANtimeSeg2;
	canBus->Init.TimeTriggeredMode = DISABLE;
	canBus->Init.AutoBusOff = DISABLE;
	canBus->Init.AutoWakeUp = DISABLE;
	canBus->Init.AutoRetransmission = DISABLE;
	canBus->Init.ReceiveFifoLocked = DISABLE;
	canBus->Init.TransmitFifoPriority = ENABLE;

	return HAL_CAN_Init(canBus);
}

/*******************************************************************************
  * @brief  Set STM32 HAL CAN filter
  * @param  ExtendedIdentifier: 0 = normal CAN identifier; 1 = extended CAN identifier
  * @param  FilterNum CAN bus filter number for this CAN bus 0...13 / 0...27
  * @param  Mask uint32_t bit mask
  * @param  Filter uint32_t CAN identifier
  * @retval success or not
  */
HAL_StatusTypeDef tNMEA2000_STM32::SetCANFilter( bool ExtendedIdentifier, uint32_t FilterNum, uint32_t Mask, uint32_t Filter )
{
	HAL_StatusTypeDef ret = HAL_ERROR;

	// For dual CAN MCU's we have 28 filter banks to share between the CAN busses
	// For single CAN SlaveStartFilterBank does nothing and we have 14 filter banks.
	// If not defined different we use filter 0 .. 13 for primary CAN bus and 14 ... 27 for secondary CAN bus
	#if !defined(SlaveStartFilterBank)
	const uint32_t SlaveStartFilterBank = 14;
	#endif

	#if defined(CAN2) // we have two CAN busses
		const int32_t TotalFilterBanks = 27;
    #elif defined(CAN1) // we have only one CAN bus
		const int32_t TotalFilterBanks = 13;
	#else // we have no CAN defined
		const int32_t TotalFilterBanks = -1;
	#endif

	int32_t FilterBank = -1;
	if (canBus->Instance == CAN1
			&& FilterNum <= TotalFilterBanks
			&& FilterNum < SlaveStartFilterBank ) {
		FilterBank = FilterNum;
	}
	else if (canBus->Instance == CAN2
			&& FilterNum <= TotalFilterBanks - SlaveStartFilterBank) {
		FilterBank = FilterNum + SlaveStartFilterBank;
	}

	if ( FilterBank >= 0
			&& IS_CAN_ALL_INSTANCE(canBus->Instance) )
	{
		CAN_FilterTypeDef sFilterConfig;

		sFilterConfig.FilterBank = FilterBank;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

		if (ExtendedIdentifier == false)
		{
			sFilterConfig.FilterMaskIdHigh = Mask << 5 & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = 0x0000; // allows both remote request and data frames
			sFilterConfig.FilterIdHigh = Filter << 5 & 0xFFFF;
			sFilterConfig.FilterIdLow =  0x0000;
		}
		else
		{ // ExtendedIdentifier == true
			sFilterConfig.FilterMaskIdHigh = Mask >> 13 & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = (Mask << 3 & 0xFFF8) | (0x1 << 2);
			sFilterConfig.FilterIdHigh = Filter >> 13 & 0xFFFF; // EXTID[28:13]
			sFilterConfig.FilterIdLow = (Filter << 3 & 0xFFF8) | (0x1 << 2); // EXTID[12:0] + IDE
		}

		sFilterConfig.FilterFIFOAssignment = CANRxFIFO;
		sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		sFilterConfig.SlaveStartFilterBank = SlaveStartFilterBank; // CAN 0: 0...13 // CAN 1: 14...27 (28 filter banks in total)

		ret = HAL_CAN_ConfigFilter(canBus, &sFilterConfig);
		DbgPrintfSWD("CAN filter bank %li mask: %lx, filter: %lx", FilterBank, Mask, Filter);
	}
	return ret;
}



/*******************************************************************************
  * @brief  returns pointer to tNMEA2000_STM32 instance from certain CAN_HandleTypeDef struct
  * @param  hcan CAN_HandleTypeDef pointer
  * @retval tNMEA2000_STM32  pointer
  */
tNMEA2000_STM32* getInstance(CAN_HandleTypeDef *hcan)
{
	tNMEA2000_STM32* instance;
	for (auto & inst : canInstances) { // iterate over all canInstances
		if (inst->canBus == hcan) {
			instance = inst;
		}
	}
	return instance;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	getInstance(hcan)->CANReadRxMailbox(hcan, CAN_RX_FIFO0);
	//NMEA2000_STM32_instance->CANReadRxMailbox(hcan);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	// Call TX Interrupt method
	//NMEA2000_STM32_instance->SendFromTxRing(); // send message with highest priority on ring buffer
	getInstance(hcan)->SendFromTxRing(); // send message with highest priority on ring buffer
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	// Call TX Interrupt method
	getInstance(hcan)->SendFromTxRing(); // send message with highest priority on ring buffer
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	// Call TX Interrupt method
	getInstance(hcan)->SendFromTxRing(); // send message with highest priority on ring buffer
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
   uint32_t errorCode = hcan->ErrorCode;

   if(errorCode & HAL_CAN_ERROR_NONE)            {DbgPrintfSWDErr("No error");}
   if(errorCode & HAL_CAN_ERROR_EWG)             {DbgPrintfSWDErr("Protocol Error Warning");}
   if(errorCode & HAL_CAN_ERROR_EPV)             {DbgPrintfSWDErr("Error Passive");}
   if(errorCode & HAL_CAN_ERROR_BOF)             {DbgPrintfSWDErr("Bus-off error");}
   if(errorCode & HAL_CAN_ERROR_STF)             {DbgPrintfSWDErr("Stuff error");}
   if(errorCode & HAL_CAN_ERROR_FOR)             {DbgPrintfSWDErr("Form error");}
   if(errorCode & HAL_CAN_ERROR_ACK)             {DbgPrintfSWDErr("Acknowledgment error");}
   if(errorCode & HAL_CAN_ERROR_BR)              {DbgPrintfSWDErr("Bit recessive error");}
   if(errorCode & HAL_CAN_ERROR_BD)              {DbgPrintfSWDErr("Bit dominant error");}
   if(errorCode & HAL_CAN_ERROR_CRC)             {DbgPrintfSWDErr("CRC error");}
   if(errorCode & HAL_CAN_ERROR_RX_FOV0)         {DbgPrintfSWDErr("Rx FIFO 0 overrun error");}
   if(errorCode & HAL_CAN_ERROR_RX_FOV1)         {DbgPrintfSWDErr("Rx FIFO 1 overrun error");}
   if(errorCode & HAL_CAN_ERROR_TX_ALST0)        {DbgPrintfSWDErr("TxMailbox 0 transmit failure due to arbitration lost");}
   if(errorCode & HAL_CAN_ERROR_TX_TERR0)        {DbgPrintfSWDErr("TxMailbox 0 transmit failure due to transmit error");}
   if(errorCode & HAL_CAN_ERROR_TX_ALST1)        {DbgPrintfSWDErr("TxMailbox 1 transmit failure due to arbitration lost");}
   if(errorCode & HAL_CAN_ERROR_TX_TERR1)        {DbgPrintfSWDErr("TxMailbox 1 transmit failure due to transmit error");}
   if(errorCode & HAL_CAN_ERROR_TX_ALST2)        {DbgPrintfSWDErr("TxMailbox 2 transmit failure due to arbitration lost");}
   if(errorCode & HAL_CAN_ERROR_TX_TERR2)        {DbgPrintfSWDErr("TxMailbox 2 transmit failure due to transmit error");}
   if(errorCode & HAL_CAN_ERROR_TIMEOUT)         {DbgPrintfSWDErr("Timeout error");}
   if(errorCode & HAL_CAN_ERROR_NOT_INITIALIZED) {DbgPrintfSWDErr("Peripheral not initialized");}
   if(errorCode & HAL_CAN_ERROR_NOT_READY)       {DbgPrintfSWDErr("Peripheral not ready");}
   if(errorCode & HAL_CAN_ERROR_NOT_STARTED)     {DbgPrintfSWDErr("Peripheral not started");}
   if(errorCode & HAL_CAN_ERROR_PARAM)           {DbgPrintfSWDErr("Parameter error");}

}
// *****************************************************************************
//	Other 'Bridge' functions

void delay(const uint32_t ms) {
	HAL_Delay(ms);
};


//*****************************************************************************
uint32_t millis(void) {
    return HAL_GetTick();
};

//*****************************************************************************
// switch printf() to the debug interface SWO (STM32 ARM debug)
int _write(int file, char *ptr, int len) {
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}
// N2kStream is using write() function
int write(char *str, int len) {
	return _write(0, str, len);
}

