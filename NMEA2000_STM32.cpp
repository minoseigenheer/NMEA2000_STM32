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


#include <cstring>
#include <string.h>
#include "main.h"

#include <NMEA2000_STM32.h>


#ifdef CAN0
//static tNMEA2000_STM32* _CAN0 = 0;
#endif
#ifdef CAN1
//static tNMEA2000_STM32* _CAN1 = 0;
#endif


//*****************************************************************************
tNMEA2000_STM32::tNMEA2000_STM32(CAN_HandleTypeDef *_N2kCan) :
		tNMEA2000(), N2kCan(_N2kCan) {

	NMEA2000_STM32_instance = this;
}


//*****************************************************************************
bool tNMEA2000_STM32::CANOpen() {

	bool ret = true;

	// Enable CAN
	if (HAL_CAN_Start(N2kCan) != HAL_OK) {
		ret = false;
	}

	// activate CAN callback 1 interrupt for NMEA2000 CAN bus
	if (HAL_CAN_ActivateNotification(N2kCan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
		ret = false;
	}

	bool ExtendedId      = true;
	uint32_t FilterNum   = 0;
	uint32_t Mask        = 0x00000000;
	uint32_t Filter      = 0x00000000;
	if (SetCANFilter( N2kCan, ExtendedId, FilterNum, Mask, Filter ) != true) {
		ret = false;
	}

	return ret;
}

//*****************************************************************************
bool tNMEA2000_STM32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent) {
	//TODO wait_sent
	uint8_t prio = (uint8_t)((id >> 26) & 0x7);
	bool ret = false;

	DisableIRQ_CAN_RX();

	bool TxMailboxesFull = HAL_CAN_GetTxMailboxesFreeLevel(N2kCan) == 0;
	bool SendFromBuffer = false;

	// If tx buffer has already some frames waiting or mailbox is full, buffer frame
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
		}
		SendFromBuffer = true;
	}

	if ( !TxMailboxesFull ) {
		if ( SendFromBuffer ) {
			ret = sendFromTxRing(prio);
		} else {
			ret = writeTxMailbox(id, len, buf, 1);
		}
		/* transmit entry accepted */
	}

	EnableIRQ_CAN_RX();

	return ret;
}

//*****************************************************************************
bool tNMEA2000_STM32::CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) {

	bool ret = false;

	DisableIRQ_CAN_TX();

	const CAN_message_t *msg = rxRing->getReadRef();
	if ( msg!=0 ) {
	    id = msg->id;
	    len = msg->len;
		if ( len > 8 ) len = 8;
	    memcpy(buf, msg->buf, len);
	    ret = true;
	}

	EnableIRQ_CAN_TX();

	return ret;

}

// *****************************************************************************
void tNMEA2000_STM32::InitCANFrameBuffers() {

  if ( MaxCANReceiveFrames == 0 ) MaxCANReceiveFrames = 32; // Use default, if not set
  if ( MaxCANReceiveFrames < 10 ) MaxCANReceiveFrames = 10; // Do not allow less than 10 - should have enough memory.
  if ( MaxCANSendFrames == 0 ) MaxCANSendFrames = 50;  // Use big enough default buffer
  if ( MaxCANSendFrames < 30 ) MaxCANSendFrames = 30; // Do not allow less than 30 - should have enough memory.

  if ( rxRing != 0 && rxRing->getSize() != MaxCANReceiveFrames ) {
    delete rxRing;
    rxRing = 0;
  }
  if ( txRing != 0 && txRing->getSize() != MaxCANSendFrames ) {
    delete rxRing;
    txRing = 0;
  }

  if ( rxRing == 0 ) rxRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANReceiveFrames, 7);
  if ( txRing == 0 ) txRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANSendFrames, 7);

}

// *****************************************************************************
bool tNMEA2000_STM32::writeTxMailbox(unsigned long id, unsigned char len, const unsigned char *buf, bool extended) {

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
	if (HAL_CAN_AddTxMessage(N2kCan, &CANTxHeader, CANTxdata, &CANTxMailbox) == HAL_OK) {
		return true;
	} else {
		return false;
	}

}

// *****************************************************************************
bool tNMEA2000_STM32::sendFromTxRing(uint8_t prio) {
	const CAN_message_t *txMsg;

	txMsg = txRing->getReadRef(prio);
	if ( txMsg != 0 ) {
		return writeTxMailbox(txMsg->id, txMsg->len, txMsg->buf, txMsg->flags.extended);
	} else {
		return false;
	}

}

// *****************************************************************************
void tNMEA2000_STM32::CANRxInterrupt(CAN_HandleTypeDef *hcan) {
	CAN_message_t *rxMsg;
	uint8_t prio;

	if (hcan == N2kCan) {
		if (HAL_CAN_GetRxMessage(hcan, RX_FIFO, &CANRxHeader, CANRxdata) == HAL_OK) {
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
		}
	}
	// I think we don't have to check the fifo fill level if we use interrups?
	// HAL_CAN_GetRxFifoFillLevel(*N2kCan, CAN_RX_FIFO1);

}

void tNMEA2000_STM32::DisableIRQ_CAN_RX() {
	if (N2kCan->Instance == CAN1) {
		NVIC_DisableIRQ(CAN1_RX1_IRQn);
	}
	else if (N2kCan->Instance == CAN2) {
		NVIC_DisableIRQ(CAN2_RX1_IRQn);
	}
}
void tNMEA2000_STM32::EnableIRQ_CAN_RX() {
	if (N2kCan->Instance == CAN1) {
		NVIC_EnableIRQ(CAN1_RX1_IRQn);
	}
	else if (N2kCan->Instance == CAN2) {
		NVIC_EnableIRQ(CAN2_RX1_IRQn);
	}
}
void tNMEA2000_STM32::DisableIRQ_CAN_TX() {
	if (N2kCan->Instance == CAN1) {
		NVIC_DisableIRQ(CAN1_TX_IRQn);
	}
	else if (N2kCan->Instance == CAN2) {
		NVIC_DisableIRQ(CAN2_TX_IRQn);
	}
}
void tNMEA2000_STM32::EnableIRQ_CAN_TX() {
	if (N2kCan->Instance == CAN1) {
		NVIC_EnableIRQ(CAN1_TX_IRQn);
	}
	else if (N2kCan->Instance == CAN2) {
		NVIC_EnableIRQ(CAN2_TX_IRQn);
	}
}


// *****************************************************************************
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// Call rxInterrupt method of the last tNMEA2000_STM32 instance.
	NMEA2000_STM32_instance->CANRxInterrupt(hcan);
}

/**
  * @brief  Set STM32 HAL CAN filter
  * @param  hcan CAN_HandleTypeDef pointer to CAN instance
  * @param  ExtendedIdentifier: 0 = normal CAN identifier; 1 = extended CAN identifier
  * @param  FilterNum CAN bus filter number for this CAN bus 0...13 / 0...27
  * @param  Mask uint32_t bit mask
  * @param  Filter uint32_t CAN identifier
  * @retval success or not
  */
bool SetCANFilter( CAN_HandleTypeDef *hcan, bool ExtendedIdentifier, uint32_t FilterNum, uint32_t Mask, uint32_t Filter )
{
	bool ret = false;

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
	if (hcan->Instance == CAN1
			&& FilterNum <= TotalFilterBanks
			&& FilterNum < SlaveStartFilterBank ) {
		FilterBank = FilterNum;
	}
	else if (hcan->Instance == CAN2
			&& FilterNum <= TotalFilterBanks - SlaveStartFilterBank) {
		FilterBank = FilterNum + SlaveStartFilterBank;
	}

	if ( FilterBank >= 0
			&& IS_CAN_ALL_INSTANCE(hcan->Instance) )
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

		sFilterConfig.FilterFIFOAssignment = 0;
		sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		sFilterConfig.SlaveStartFilterBank = SlaveStartFilterBank; // CAN 0: 0...13 // CAN 1: 14...27 (28 filter banks in total)

		if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
			ret = true;
		}
	}
	return ret;
}

