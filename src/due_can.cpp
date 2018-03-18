/*
  Copyright (c) 2013 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

//----------------------------------------------------------------------------------------------------------------------
#include "due_can.h"

//----------------------------------------------------------------------------------------------------------------------
// Set the debugging interface for autobaud.
#if 0
#	define AUTOBAUD_DEBUG(...)  SerialUSB.print(__VA_ARGS__)
#	define DEBUGF(...)          SerialUSB.print(__VA_ARGS__)
#else
#	define AUTOBAUD_DEBUG(...)  do { } while(0)
#	define DEBUGF(...)          do { } while(0)
#endif

//----------------------------------------------------------------------------------------------------------------------
// Instantiate the two canbus adapters
//
CANRaw  Can0(CAN0, CAN0_EN) ;
CANRaw  Can1(CAN1, CAN1_EN) ;

//+=====================================================================================================================
// pCan .. either "CAN0" or "CAN1"
// En   .. the pin on the arduino connected to the transceiver enable pin
//
CANRaw::CANRaw (Can* pCan,  uint32_t En) : CAN_COMMON(8)
{
	m_pCan    = pCan;
	nIRQ      = (m_pCan == CAN0) ? CAN0_IRQn : CAN1_IRQn;
	enablePin = En;
	bigEndian = false;
	busSpeed  = 0;

	for (int  i = 0;  i < SIZE_LISTENERS;  listener[i++] = NULL) ;

	rx_frame_buff = 0;
	tx_frame_buff = 0;
	numTXBoxes    = 1;

	sizeRxBuffer = SIZE_RX_BUFFER;
	sizeTxBuffer = SIZE_TX_BUFFER;

	disable_autobaud_listen_mode();
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;

	// Initialize all message box spesific ring buffers to 0.
	for (int  i = 0;  i < getNumMailBoxes();  txRings[i++] = 0) ;
}

//+=====================================================================================================================
uint32_t  CANRaw::set_baudrate (uint32_t ul_baudrate)
{
	static uint32_t    ul_mck = SystemCoreClock;

	uint8_t            uc_tq;
	uint8_t            uc_prescale;
	uint32_t           ul_mod;
	uint32_t           ul_cur_mod;
	can_bit_timing_t*  p_bit_time;

	/* Check whether the baudrate prescale will be greater than the max divide value. */
	if ( ((ul_mck + ((ul_baudrate * CAN_MAX_TQ_NUM) - 1)) / (ul_baudrate * CAN_MAX_TQ_NUM)) > CAN_BAUDRATE_MAX_DIV )
		return 0;

	/* Check whether the input MCK is too small. */
	if (ul_mck  < (ul_baudrate * CAN_MIN_TQ_NUM))  return 0 ;

	/* Initialize it as the minimum Time Quantum. */
	uc_tq = CAN_MIN_TQ_NUM;

	/* Initialize the remainder as the max value. When the remainder is 0, get the right TQ number. */
	ul_mod = 0xFFFFFFFF;
	/* Find out the approximate Time Quantum according to the baudrate. */
	for (uint8_t  i = CAN_MIN_TQ_NUM;  i <= CAN_MAX_TQ_NUM;  i++) {
		if ((ul_mck / (ul_baudrate * i)) <= CAN_BAUDRATE_MAX_DIV) {
			ul_cur_mod = ul_mck % (ul_baudrate * i);

			if (ul_cur_mod < ul_mod) {
				ul_mod = ul_cur_mod;
				uc_tq  = i;
				if (!ul_mod)  break ;
			}
		}
	}

	/* Calculate the baudrate prescale value. */
	uc_prescale = ul_mck / (ul_baudrate * uc_tq);

	/* Get the right CAN BIT Timing group. */
	p_bit_time = (can_bit_timing_t*)&can_bit_time[uc_tq - CAN_MIN_TQ_NUM];

	/* Before modifying the CANBR register, disable the CAN controller. */
	//can_disable(m_pCan);
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;

	/* Write into the CAN baudrate register. */
	m_pCan->CAN_BR = CAN_BR_PHASE2(p_bit_time->uc_phase2   - 1)
	               | CAN_BR_PHASE1(p_bit_time->uc_phase1   - 1)
	               | CAN_BR_PROPAG(p_bit_time->uc_prog     - 1)
	               | CAN_BR_SJW   (p_bit_time->uc_sjw      - 1)
	               | CAN_BR_BRP   (            uc_prescale - 1) ;
	return 1;
}

//+=====================================================================================================================
// returns : 0 .. failed ; >0 .. active baudrate
uint32_t  CANRaw::beginAutoSpeed ()
{
	// 0 terminated List of speeds to check
	uint32_t  speed[] = {CAN_BPS_250K, CAN_BPS_500K,  CAN_BPS_1000K,
	                     CAN_BPS_125K, CAN_BPS_33333, CAN_BPS_50K,   CAN_BPS_800K,
	                     0};
	uint32_t  spd;


	// Go in to listen-only mode, so we don't clobber the bus with a wrong speed setting
	enable_autobaud_listen_mode();

	AUTOBAUD_DEBUG("\nTrying CAN rate: ");
	for (int  i = 0;  speed[i];  i++) {
		if (i)  AUTOBAUD_DEBUG(", ") ;
		AUTOBAUD_DEBUG(speed[i]);

		if ( !(spd = init(speed[i])) ) {
			AUTOBAUD_DEBUG(" [BadSpeed]");
			continue;
		}

		// !!! why initialise 7 filters??
		//for (int  filter = 0;  filter < 3;  filter++)  setRXFilter(filter, 0, 0, true) ;
		//for (int  filter = 3;  filter < 7;  filter++)  setRXFilter(filter, 0, 0, false) ;
		setRXFilter(0, 0, 0, true);
		setRXFilter(1, 0, 0, false);

		for (int  waiting = 0;  waiting < 100;  delay(6), waiting++)
			if (rx_avail()) {
				CAN_FRAME  thisFrame;
				read(thisFrame);
				break;
			}

		if (numRxFrames > 0)  {
			AUTOBAUD_DEBUG(" [Success]\n");
			disable_autobaud_listen_mode(); //the default is to not be in listen only
			reset_all_mailbox(); //return mailboxes to default state which is to let nothing through yet
			init(spd);
			return spd; //return the speed that succeeded

		} else  AUTOBAUD_DEBUG(" [x]") ;
	}
	AUTOBAUD_DEBUG(" - FAIL: Check bus connection\n");
	disable();
	return 0;
}

//+=====================================================================================================================
void  CANRaw::setMailBoxTxBufferSize (uint8_t mbox,  uint16_t size)
{
	if ((mbox >= getNumMailBoxes()) || txRings[mbox])  return ;

	volatile  CAN_FRAME*  buf = new CAN_FRAME[size];

	txRings[mbox] = new ringbuffer_t;
	initRingBuffer(*(txRings[mbox]), buf, size);
}

//+=====================================================================================================================
// Initialize dynamically sized buffers.
//
void  CANRaw::initializeBuffers ()
{
	if (isInitialized())  return ;

	DEBUGF("Initialize buffers");
	// set up the transmit and receive ring buffers
	if (!tx_frame_buff)  tx_frame_buff = new CAN_FRAME[sizeTxBuffer] ;
	if (!rx_frame_buff)  rx_frame_buff = new CAN_FRAME[sizeRxBuffer] ;

	initRingBuffer(txRing, tx_frame_buff, sizeTxBuffer);
	initRingBuffer(rxRing, rx_frame_buff, sizeRxBuffer);
}

//+=====================================================================================================================
// PMC clock for CAN peripheral should be enabled before calling this function.
//
uint32_t  CANRaw::init (uint32_t ul_baudrate)
{
	uint32_t ul_flag;
	uint32_t ul_tick;
	uint32_t ul_status;

	initializeBuffers();
	m_pCan->CAN_MR &= ~CAN_MR_CANEN; //immediately disable the CAN hardware if it had previously been enabled

	ul_status = m_pCan->CAN_SR; // read the status register just to be sure it gets cleared out
	(void)ul_status;            // avoid compiler warnings

	numBusErrors = 0;
	numRxFrames  = 0;

	//initialize all function pointers to null
	for (int  i = 0;  i < getNumMailBoxes()+1;  cbCANFrame[i++] = 0) ;

	// Arduino 1.5.2 doesn't init canbus so make sure to do it here.
#	ifdef ARDUINO152
		PIO_Configure(PIOA,PIO_PERIPH_A, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0, PIO_DEFAULT);
		PIO_Configure(PIOB,PIO_PERIPH_A, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1, PIO_DEFAULT);
#	endif

	if (m_pCan == CAN0)  pmc_enable_periph_clk(ID_CAN0) ;
	if (m_pCan == CAN1)  pmc_enable_periph_clk(ID_CAN1) ;

	if (enablePin != 255) {
		pinMode(enablePin, OUTPUT);
		digitalWrite(enablePin, HIGH);
	}

	/* Initialize the baudrate for CAN module. */
	if ( !(ul_flag = set_baudrate(ul_baudrate)) )  return 0 ;

	/* Reset the CAN eight message mailbox. */
	reset_all_mailbox();

	//Also disable all interrupts by default
	disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);

	//By default use one mailbox for TX
	setNumTXBoxes(numTXBoxes);

	/* Enable the CAN controller. */
	enable();

	/* Wait until the CAN is synchronized with the bus activity. */
	ul_flag = 0;
	ul_tick = 0;
	while (!(ul_flag & CAN_SR_WAKEUP) && (ul_tick < CAN_TIMEOUT)) {
		ul_flag = m_pCan->CAN_SR;
		ul_tick++;
	}

	// set a fairly low priority so almost anything can preempt.
	// this has the effect that most anything can interrupt our interrupt handler
	// that's a good thing because the interrupt handler is long and complicated
	// and can send callbacks into user code which could also be long and complicated.
	// But, keep in mind that user code in callbacks runs in interrupt context
	// but can still be preempted at any time.
	NVIC_SetPriority(nIRQ, 12);

	NVIC_EnableIRQ(nIRQ); // tell the nested interrupt controller to turn on our interrupt

	/* Timeout or the CAN module has been synchronized with the bus. */
	if (CAN_TIMEOUT == ul_tick)  return 0 ;
	else                         return (busSpeed = ul_baudrate) ;
}

//+=====================================================================================================================
/*
 * \brief Initializes mailboxes to the requested mix of RX and TX boxes
 * \param txboxes How many of the 8 boxes should be used for TX
 * \retval number of tx boxes set.
 */
int  CANRaw::setNumTXBoxes (int txboxes)
{
	int  c;

	if (txboxes > getNumMailBoxes())  txboxes = getNumMailBoxes() ;
	if (txboxes < 0)                  txboxes = 0 ;
	numTXBoxes = txboxes;

	//Inialize RX boxen
	for (c = 0; c < getNumRxBoxes(); c++) {
		mailbox_set_mode(c, CAN_MB_RX_MODE);
		mailbox_set_id(c, 0x0, false);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}

	//Initialize TX boxen
	for (c = getFirstTxBox(); c < getNumMailBoxes(); c++) {
		mailbox_set_mode(c, CAN_MB_TX_MODE);
		mailbox_set_priority(c, 10);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}

	return (numTXBoxes);
}

//+=====================================================================================================================
/*
 * \brief Initialize the specified ring buffer.
 * \param ring - ring buffer to initialize.
 * \param buffer - buffer to use for storage.
 * \param size - size of the buffer in bytes.
 * \retval None.
 */
void  CANRaw::initRingBuffer (ringbuffer_t& ring,  volatile CAN_FRAME* buffer,  uint16_t size)
{
// !!! should this be IRQ locked?
	ring.buffer = buffer;
	ring.size   = size;
	ring.head   = 0;
	ring.tail   = 0;
}

//+=============================================================================
bool  CANRaw::addToRingBuffer (ringbuffer_t& ring,  const CAN_FRAME& msg)
{
	uint16_t nextEntry;

// !!! should this be IRQ locked?
	nextEntry = (ring.head + 1) % ring.size;

	/* check if the ring buffer is full */
	if (nextEntry == ring.tail)  return false ;

	/* add the element to the ring */
	// How dodgy is this? Casting off the volatile :-o
	//memcpy((void*)&ring.buffer[ring.head], (void *)&msg, sizeof (CAN_FRAME));
	memcpy((void*)&ring.buffer[ring.head], &msg, sizeof(CAN_FRAME));

	/* bump the head to point to the next free entry */
	ring.head = nextEntry;

	return (true);
}

//+=============================================================================
bool  CANRaw::removeFromRingBuffer (ringbuffer_t& ring,  CAN_FRAME& msg)
{
	if (isRingBufferEmpty(ring))  return false ;

// !!! should this be IRQ locked?
	// How dodgy is this? Casting off the volatile :-o
	memcpy(&msg, (const void*)&ring.buffer[ring.tail], sizeof(CAN_FRAME));

	ring.tail = (ring.tail + 1) % ring.size;

	return true;
}

//+=============================================================================
uint16_t  CANRaw::ringBufferCount (ringbuffer_t& ring)
{
// !!! should this be IRQ locked?
	if (ring.tail == ring.head)  return 0 ;
	if (ring.tail  < ring.head)  return ring.head - ring.tail ;
	else                         return ring.size - ring.tail + ring.head ;
}

//+=====================================================================================================================
void  CANRaw::setListenOnlyMode (bool state)
{
	if (state)  enable_autobaud_listen_mode() ;
	else        disable_autobaud_listen_mode() ;
}

//+=====================================================================================================================
void  CANRaw::enable ()
{
	m_pCan->CAN_MR |= CAN_MR_CANEN;

	if (enablePin != 255)  digitalWrite(enablePin, HIGH) ;
}

//+=============================================================================
void  CANRaw::disable ()
{
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;

	if (enablePin != 255)  digitalWrite(enablePin, LOW) ;
}

//+=====================================================================================================================
void  CANRaw::setModeBit (uint32_t bits)
{
/*
	uint32_t savedMR = m_pCan->CAN_MR;  // Save the MR state

	m_pCan->CAN_MR &= ~CAN_MR_CANEN;    // Disable the CAN controller
	m_pCan->CAN_MR |= bit;              // Set the requested bit

	if (savedMR & CAN_MR_CANEN) {       // If CAN was enabled
		savedMR        |= bit;          //
		m_pCan->CAN_MR  = savedMR;
	}
*/
	if (m_pCan->CAN_MR & CAN_MR_CANEN) {        // If CAN is enabled
		m_pCan->CAN_MR &= ~CAN_MR_CANEN;        //   Disable the CAN controller
		m_pCan->CAN_MR |= CAN_MR_CANEN | bits;  //   Enable the CAN controller - with the requested bits set
	} else {                                    // else CAN is disabled
		m_pCan->CAN_MR |= bits;                 //   Set the requested bits
	}
}



//+=============================================================================
void  CANRaw::unsetModeBit (uint32_t bits)
{
/*
	uint32_t savedMR = m_pCan->CAN_MR;

	m_pCan->CAN_MR &= ~CAN_MR_CANEN;
	m_pCan->CAN_MR &= ~bit;

	if (savedMR & CAN_MR_CANEN) {
		savedMR        &= ~bit;
		m_pCan->CAN_MR  = savedMR;
	}
*/
	if (m_pCan->CAN_MR & CAN_MR_CANEN) {  // If CAN is enabled
		m_pCan->CAN_MR &= ~CAN_MR_CANEN;  //   Disable the CAN controller
		m_pCan->CAN_MR &= ~bits;          //   Reset the requested bits
		m_pCan->CAN_MR |= CAN_MR_CANEN;   //   Enable the CAN controller
	} else {                              // else CAN is disabled
		m_pCan->CAN_MR &= ~bits;          //   Reset the requested bits
	}

}

//+=====================================================================================================================
// Setting and unsetting of various control bits
//
// "Overload frames" are generated after a successful Rx to an Rx-mailbox (Producer or Consumer)
void  CANRaw::enable_overload_frame        ()  {  setModeBit  (CAN_MR_OVL   );  }
void  CANRaw::disable_overload_frame       ()  {  unsetModeBit(CAN_MR_OVL   );  }

void  CANRaw::enable_low_power_mode        ()  {  setModeBit  (CAN_MR_LPM   );  }
void  CANRaw::disable_low_power_mode       ()  {  unsetModeBit(CAN_MR_LPM   );  }

void  CANRaw::enable_autobaud_listen_mode  ()  {  setModeBit  (CAN_MR_ABM   );  }
void  CANRaw::disable_autobaud_listen_mode ()  {  unsetModeBit(CAN_MR_ABM   );  }

void  CANRaw::enable_time_triggered_mode   ()  {  setModeBit  (CAN_MR_TTM   );  }
void  CANRaw::disable_time_triggered_mode  ()  {  unsetModeBit(CAN_MR_TTM   );  }

void  CANRaw::enable_timer_freeze          ()  {  setModeBit  (CAN_MR_TIMFRZ);  }
void  CANRaw::disable_timer_freeze         ()  {  unsetModeBit(CAN_MR_TIMFRZ);  }

void  CANRaw::enable_tx_repeat             ()  {  unsetModeBit(CAN_MR_DRPT  );  }  // repeat = !don't_repeat
void  CANRaw::disable_tx_repeat            ()  {  setModeBit  (CAN_MR_DRPT  );  }

void  CANRaw::set_timestamp_capture_point (uint32_t ul_flag)
{
	if (ul_flag)  setModeBit  (CAN_MR_TEOF) ;  // 1 : timestamp is START of frame
	else          unsetModeBit(CAN_MR_TEOF) ;  // 0 : timestamp is  END  of frame
}

//+=====================================================================================================================
/**
 * \brief Configure CAN Controller reception synchronization stage.
 * \param ul_stage The reception stage to be configured.
 * \note This is just for debug purpose only.
 */
void  CANRaw::set_rx_sync_stage (uint32_t ul_stage)
{
	m_pCan->CAN_MR = (m_pCan->CAN_MR & ~CAN_MR_RXSYNC_Msk) | ul_stage;
}

//+=====================================================================================================================
// Interrupt control
//
void      CANRaw::enable_interrupt   (uint32_t dw_mask)  {  m_pCan->CAN_IER = dw_mask; }
void      CANRaw::disable_interrupt  (uint32_t dw_mask)  {  m_pCan->CAN_IDR = dw_mask; }
uint32_t  CANRaw::get_interrupt_mask ()                  {  return m_pCan->CAN_IMR;    }

//+=====================================================================================================================
// Getters
//
uint32_t  CANRaw::get_status               ()  {  return m_pCan->CAN_SR;                                 }
uint16_t  CANRaw::get_internal_timer_value ()  {  return m_pCan->CAN_TIM;                                }
uint16_t  CANRaw::get_timestamp_value      ()  {  return m_pCan->CAN_TIMESTP;                            }
uint8_t   CANRaw::get_tx_error_cnt         ()  {  return (uint8_t)(m_pCan->CAN_ECR >> CAN_ECR_TEC_Pos);  }
uint8_t   CANRaw::get_rx_error_cnt         ()  {  return (uint8_t)(m_pCan->CAN_ECR >> CAN_ECR_REC_Pos);  }

//+=====================================================================================================================
// If the internal timer counter is frozen, this function re-enables it.
//
void  CANRaw::reset_internal_timer ()
{
	m_pCan->CAN_TCR |= CAN_TCR_TIMRST;
}

//+=====================================================================================================================
/**
 * \brief Send global transfer request.
 * \param uc_mask Mask for mailboxes that are requested to transfer.
 */
void  CANRaw::global_send_transfer_cmd (uint8_t uc_mask)
{
	m_pCan->CAN_TCR = uc_mask & GLOBAL_MAILBOX_MASK;
}

//+=====================================================================================================================
/**
 * \brief Send global abort request.
 * \param uc_mask Mask for mailboxes that are requested to abort.
 */
void  CANRaw::global_send_abort_cmd (uint8_t uc_mask)
{
	m_pCan->CAN_ACR = (m_pCan->CAN_ACR & ((uint32_t)~GLOBAL_MAILBOX_MASK)) | uc_mask;
}

//+=====================================================================================================================
/**
 * \brief Configure the timemark for the mailbox.
 * \param uc_index Indicate which mailbox is to be configured.
 * \param us_cnt   The timemark to be set.
 * \note The timemark is active in Time Triggered mode only.
 */
void  CANRaw::mailbox_set_timemark (uint8_t uc_index,  uint16_t us_cnt)
{
	// !!! OK, we really need to be returning an error and not just grabbing the last mailbox
	// !!! This (worryingly) might explain why all the code I keep looking at seems to avoid the last mailbox
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;

	m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR & ((uint32_t)~TIMEMARK_MASK)) | us_cnt;
}

//+=====================================================================================================================
/**
 * \brief Get status of the mailbox.
 * \param uc_index Indicate which mailbox is to be read.
 * \retval The mailbox status.
 */
uint32_t  CANRaw::mailbox_get_status (uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	return m_pCan->CAN_MB[uc_index].CAN_MSR;
}

//+=====================================================================================================================
/**
 * \brief Send single mailbox transfer request.
 * \param uc_index Indicate which mailbox is to be configured.
 */
void  CANRaw::mailbox_send_transfer_cmd (uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MTCR;
}

//+=====================================================================================================================
/**
 * \brief Send single mailbox abort request.
 * \param uc_index Indicate which mailbox is to be configured.
 */
void  CANRaw::mailbox_send_abort_cmd (uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MACR;
}

//+=====================================================================================================================
/**
 * \brief Initialize the mailbox to a default, known state.
 * \param p_mailbox Pointer to a CAN mailbox instance.
 */
void  CANRaw::mailbox_init (uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MMR = 0;
	m_pCan->CAN_MB[uc_index].CAN_MAM = 0;
	m_pCan->CAN_MB[uc_index].CAN_MID = 0;
	m_pCan->CAN_MB[uc_index].CAN_MDL = 0;
	m_pCan->CAN_MB[uc_index].CAN_MDH = 0;
	m_pCan->CAN_MB[uc_index].CAN_MCR = 0;
}

//+=====================================================================================================================
/**
 * \brief Reset the eight mailboxes.
 * \param m_pCan Pointer to a CAN peripheral instance.
 */
void  CANRaw::reset_all_mailbox ()
{
	for (int  i = 0;  i < CANMB_NUMBER;  mailbox_init(i++) ) ;
}

//+=====================================================================================================================
void  CANRaw::setBigEndian (bool end)
{
	bigEndian = end;
}

//+=====================================================================================================================
void  CANRaw::setWriteID (uint32_t id)
{
	write_id = id;
}

//+=====================================================================================================================
template <typename t>
void  CANRaw::write (t inputValue)
{
	CAN_FRAME  tempFrame;
	uint8_t*   buff     = (uint8_t*)inputValue;
	int        thisSize = (sizeof(t) > 8) ? 8 : sizeof(t) ;

	if (!bigEndian)
		for (int  i = 0;  i < thisSize;  i++)
			tempFrame.data.bytes[i] = buff[i];

	else  //reverse byte order. The M3 is in little endian so this causes big endian order
		for (int  i = 0;  i < thisSize;  i++)
			tempFrame.data.bytes[i] = buff[thisSize - i - 1];

	tempFrame.id     = this->write_id;
	tempFrame.length = thisSize;

	if (this->write_id > 0x7FF)  tempFrame.extended = true ;
	else                         tempFrame.extended = false ;

	sendFrame(tempFrame);
}

//+=====================================================================================================================
void  CANRaw::writeTxRegisters (const CAN_FRAME& txFrame,  uint8_t mb)
{
	mailbox_set_id      (mb, txFrame.id, txFrame.extended);
	mailbox_set_datalen (mb, txFrame.length);
	mailbox_set_rtr     (mb, txFrame.rtr);
	mailbox_set_priority(mb, txFrame.priority);

	for (uint8_t  cnt = 0;  cnt < 8;  cnt++)
		mailbox_set_databyte(mb, cnt, txFrame.data.bytes[cnt]);

	global_send_transfer_cmd((0x1u << mb));
}

//+=====================================================================================================================
/**
 * \brief Send a frame out of this canbus port
 * \param txFrame The filled out frame structure to use for sending
 * \note Will do one of two things - 1. Send the given frame out of the first available mailbox
 * or 2. queue the frame for sending later via interrupt. Automatically turns on TX interrupt
 * if necessary.
 * Returns whether sending/queueing succeeded. Will not smash the queue if it gets full.
 */
bool  CANRaw::sendFrame (CAN_FRAME& txFrame)
{
	bool result = false;

	irqLock();
	{
		// If there is nothing buffered, try to find free mailbox
		if (isRingBufferEmpty(txRing)) {
			for (uint8_t mbox = 0; mbox < 8; mbox++) {
				// Is this mailbox set up as a TX box?
				if ( (((m_pCan->CAN_MB[mbox].CAN_MMR >> 24) & 7) == CAN_MB_TX_MODE)
				     // ...and also available (ie. not sending anything)?
				     && (usesGlobalTxRing(mbox)
				     && (m_pCan->CAN_MB[mbox].CAN_MSR & CAN_MSR_MRDY)) )
				{
					writeTxRegisters(txFrame,mbox);
					enable_interrupt(0x01u << mbox); //enable the TX interrupt for this box
					result = true; //we've sent it. mission accomplished.
					break; //no need to keep going. We sent our message
				}
			}
		}

		// No free mailbox was found above
		// So, queue the frame if possible.
		// But, don't increment the tail if it would smash into the head and kill the queue.
		if (!result)  result = addToRingBuffer(txRing, txFrame) ;
	}
	irqRelease();

	return result;
}

//+=====================================================================================================================
/**
 * \brief Send a frame out of this canbus port
 * \param txFrame The filled out frame structure to use for sending
 * \note Will do one of two things - 1. Send the given frame out of the first available mailbox
 * or 2. queue the frame for sending later via interrupt. Automatically turns on TX interrupt
 * if necessary.
 * Returns whether sending/queueing succeeded. Will not smash the queue if it gets full.
 */
bool  CANRaw::sendFrame (CAN_FRAME& txFrame,  uint8_t mbox)
{
	bool  result = false;

	if (!isTxBox(mbox))  return result ;

	irqLock();
	{
		if ( (!txRings[mbox] || isRingBufferEmpty(*txRings[mbox]))
		     && (m_pCan->CAN_MB[mbox].CAN_MSR & CAN_MSR_MRDY) )
		{
			writeTxRegisters(txFrame,mbox);
			enable_interrupt(0x01u << mbox); //enable the TX interrupt for this box
			result = true; //we've sent it. mission accomplished.
		}

		if (!result && txRings[mbox])  result = addToRingBuffer(*txRings[mbox], txFrame) ;
	}
	irqRelease();

	return result;
}

//+=====================================================================================================================
/**
 * \brief Read a frame from out of the mailbox and into a software buffer
 * \param uc_index which mailbox to read
 * \param rxframe Pointer to a receive frame structure which we'll fill out
 * \retval Different CAN mailbox transfer status.
 */
uint32_t  CANRaw::mailbox_read (uint8_t uc_index,  volatile CAN_FRAME* rxframe)
{
	uint32_t  ul_status;
	uint32_t  ul_retval;
	uint32_t  ul_id;
	uint32_t  ul_datal, ul_datah;

	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	ul_retval = 0;
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;

	/* Check whether there is overwriting happening in Receive with Overwrite mode,
	   or there're messages lost in Receive mode. */
	if ((ul_status & CAN_MSR_MRDY) && (ul_status & CAN_MSR_MMI))
		ul_retval = CAN_MAILBOX_RX_OVER;

	ul_id = m_pCan->CAN_MB[uc_index].CAN_MID;
	if ((ul_id & CAN_MID_MIDE) == CAN_MID_MIDE) { //extended id
		rxframe->id       = ul_id & 0x1FFFFFFFu;
		rxframe->extended = true;

	} else { //standard ID
		rxframe->id       = (ul_id  >> CAN_MID_MIDvA_Pos) & 0x7ffu;
		rxframe->extended = false;
	}

	rxframe->fid    = m_pCan->CAN_MB[uc_index].CAN_MFID;
	rxframe->length = (ul_status & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
	rxframe->time   = (ul_status & CAN_MSR_MTIMESTAMP_Msk);
	rxframe->rtr    = (m_pCan->CAN_MB[uc_index].CAN_MSR & CAN_MSR_MRTR) ? 1 : 0 ;
	ul_datal        = m_pCan->CAN_MB[uc_index].CAN_MDL;
	ul_datah        = m_pCan->CAN_MB[uc_index].CAN_MDH;

	rxframe->data.high = ul_datah;
	rxframe->data.low  = ul_datal;

	/* Read the mailbox status again to check whether the software needs to re-read mailbox data register. */
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;
	if (ul_status & CAN_MSR_MMI)  ul_retval |= CAN_MAILBOX_RX_NEED_RD_AGAIN ;
	else                          ul_retval |= CAN_MAILBOX_TRANSFER_OK ;

	/* Enable next receive process. */
	mailbox_send_transfer_cmd(uc_index);

	return ul_retval;
}

//+=====================================================================================================================
/**
 * \brief Sets the ID portion of the given mailbox
 * \param uc_index The mailbox to set (0-7)
 * \param id The ID to set (11 or 29 bit)
 * \param extended Boolean indicating if this ID should be designated as extended
 *
 */
void  CANRaw::mailbox_set_id (uint8_t uc_index,  uint32_t id,  bool extended)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	if (extended)  m_pCan->CAN_MB[uc_index].CAN_MID = id | CAN_MID_MIDE ;
	else           m_pCan->CAN_MB[uc_index].CAN_MID = CAN_MID_MIDvA(id);
}

//+=====================================================================================================================
/**
 * \brief Get ID currently associated with a given mailbox
 * \param uc_index The mailbox to get the ID from (0-7)
 * \retval The ID associated with the mailbox
 */
uint32_t  CANRaw::mailbox_get_id (uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	if (m_pCan->CAN_MB[uc_index].CAN_MID & CAN_MID_MIDE)
		return m_pCan->CAN_MB[uc_index].CAN_MID;
	else
		return (m_pCan->CAN_MB[uc_index].CAN_MID >> CAN_MID_MIDvA_Pos) & 0x7ffu;
}

//+=====================================================================================================================
/**
 * \brief Set the transmission priority for given mailbox
 * \param uc_index The mailbox to use
 * \param pri The priority to set (0-15 in descending priority)
 */
void  CANRaw::mailbox_set_priority (uint8_t uc_index,  uint8_t pri)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR & ~CAN_MMR_PRIOR_Msk)
	                                   | (pri << CAN_MMR_PRIOR_Pos);
}

//+=====================================================================================================================
/**
 * \brief Set mask for RX on the given mailbox
 * \param uc_index The mailbox to use
 * \param mask The mask to set
 * \param ext Whether this should be an extended mask or not
 */
void  CANRaw::mailbox_set_accept_mask (uint8_t uc_index,  uint32_t mask,  bool ext)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	if (ext) {
		m_pCan->CAN_MB[uc_index].CAN_MAM  = mask | CAN_MAM_MIDE;
		m_pCan->CAN_MB[uc_index].CAN_MID |= CAN_MAM_MIDE;

	} else {
		m_pCan->CAN_MB[uc_index].CAN_MAM  = CAN_MAM_MIDvA(mask);
		m_pCan->CAN_MB[uc_index].CAN_MID &= ~CAN_MAM_MIDE;
	}
}

//+=====================================================================================================================
/**
 * \brief Set the mode of the given mailbox
 * \param uc_index Which mailbox to set (0-7)
 * \param mode The mode to set mailbox to
 * \Note Modes: 0 = Disabled, 1 = RX, 2 = RX with overwrite, 3 = TX, 4 = consumer 5 = producer
 */
void  CANRaw::mailbox_set_mode (uint8_t uc_index,  uint8_t mode)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	if (mode > 5)  mode = 0 ;  // set disabled on invalid mode  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR & ~CAN_MMR_MOT_Msk)
	                                   | (mode << CAN_MMR_MOT_Pos);
}

//+=====================================================================================================================
/**
 * \brief Get current mode of given mailbox
 * \param uc_index Which mailbox to retrieve mode from (0-7)
 * \retval Mode of mailbox
 */
uint8_t  CANRaw::mailbox_get_mode (uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	return (uint8_t)((m_pCan->CAN_MB[uc_index].CAN_MMR >> CAN_MMR_MOT_Pos) & 0x07);
}

//+=====================================================================================================================
void  CANRaw::mailbox_set_databyte (uint8_t uc_index,  uint8_t bytepos,  uint8_t val)
{
/*
	uint8_t   shift;    // how many bits to shift
	uint32_t  working;  // working copy of the relevant data int

	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	if (bytepos > 7)  bytepos = 7 ;  // !!!
	shift = 8 * (bytepos & 0x03); //how many bits to shift up into position

	if (bytepos < 4) { //low data block
		working  = m_pCan->CAN_MB[uc_index].CAN_MDL & ~(0xFF << shift); //mask out where we have to be
		working |= (val << shift);
		m_pCan->CAN_MB[uc_index].CAN_MDL = working;

	} else { //high data block
		working  = m_pCan->CAN_MB[uc_index].CAN_MDH & ~(0xFF << shift); //mask out where we have to be
		working |= (val << shift);
		m_pCan->CAN_MB[uc_index].CAN_MDH = working;
	}
*/
	int     shift;
	RwReg*  dreg;

	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!
	if (bytepos > 7)  bytepos = 7 ;  // !!!

	shift = (bytepos & 0x03) * 8;
	dreg  = (bytepos & 0x04) ? &(m_pCan->CAN_MB[uc_index].CAN_MDH)
	                         : &(m_pCan->CAN_MB[uc_index].CAN_MDL) ;
	*dreg = (*dreg & ~(0xFF << shift)) | (val << shift);
}

//+=============================================================================
void  CANRaw::mailbox_set_datal (uint8_t uc_index,  uint32_t val)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MDL = val;
}

//+=============================================================================
void  CANRaw::mailbox_set_datah (uint8_t uc_index,  uint32_t val)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MDH = val;
}

//+=============================================================================
void  CANRaw::mailbox_set_datalen (uint8_t uc_index,  uint8_t dlen)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!
	if (dlen > 8)  dlen = 8 ;  // !!!

	m_pCan->CAN_MB[uc_index].CAN_MCR =
		(m_pCan->CAN_MB[uc_index].CAN_MCR & ~CAN_MCR_MDLC_Msk) | CAN_MCR_MDLC(dlen);
}

//+=====================================================================================================================
void  CANRaw::mailbox_set_rtr (uint8_t uc_index,  uint8_t rtr)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;  // !!!

	if (rtr)  m_pCan->CAN_MB[uc_index].CAN_MSR |=  CAN_MSR_MRTR;
	else      m_pCan->CAN_MB[uc_index].CAN_MSR &= ~CAN_MSR_MRTR;
}

//+=====================================================================================================================
// Returns: CAN_MAILBOX_NOT_READY  or  CAN_MAILBOX_TRANSFER_OK
//
uint32_t  CANRaw::mailbox_tx_frame (uint8_t uc_index)
{
	// !!! LOL, no bounds chekcing on the input pram

	/* Read the mailbox status firstly to check whether the mailbox is ready or not. */
	if (!(m_pCan->CAN_MB[uc_index].CAN_MSR & CAN_MSR_MRDY))  return CAN_MAILBOX_NOT_READY ;

	/* Set the MBx bit in the Transfer Command Register to send out the remote frame. */
	global_send_transfer_cmd(1 << uc_index);

	return CAN_MAILBOX_TRANSFER_OK;
}

//+=====================================================================================================================
uint16_t  CANRaw::available ()
{
	uint16_t val;

	irqLock();
	{
		val = ringBufferCount(rxRing);
	}
	irqRelease();

	return val;
}

//+=====================================================================================================================
bool  CANRaw::rx_avail ()
{
	bool result;

	irqLock();
	{
		result = !isRingBufferEmpty(rxRing);
	}
	irqRelease();

	return result;
}

//+=====================================================================================================================
// returns : 0 = nothing waiting ; 1 = frame retrieved
// !!! surely better off returing a bool?
uint32_t  CANRaw::get_rx_buff (CAN_FRAME& msg)
{
	uint32_t result;

	irqLock();
	{
		result = removeFromRingBuffer(rxRing, msg) ? 1 : 0;
	}
	irqRelease();

	return result;
}

//+=====================================================================================================================
void  CANRaw::interruptHandler ()
{
	uint32_t  ul_status = m_pCan->CAN_SR ;  // Get interrupt Status Register

	if (ul_status & CAN_SR_MB0)     mailbox_int_handler(0, ul_status) ;
	if (ul_status & CAN_SR_MB1)     mailbox_int_handler(1, ul_status) ;
	if (ul_status & CAN_SR_MB2)     mailbox_int_handler(2, ul_status) ;
	if (ul_status & CAN_SR_MB3)     mailbox_int_handler(3, ul_status) ;
	if (ul_status & CAN_SR_MB4)     mailbox_int_handler(4, ul_status) ;
	if (ul_status & CAN_SR_MB5)     mailbox_int_handler(5, ul_status) ;
	if (ul_status & CAN_SR_MB6)     mailbox_int_handler(6, ul_status) ;
	if (ul_status & CAN_SR_MB7)     mailbox_int_handler(7, ul_status) ;

	if (ul_status & CAN_SR_ERRA)    numBusErrors++ ;  // error active
	if (ul_status & CAN_SR_WARN)    {} ;              // warning limit
	if (ul_status & CAN_SR_ERRP)    {} ;              // error passive
	if (ul_status & CAN_SR_BOFF)    numBusErrors++ ;  // bus off
	if (ul_status & CAN_SR_SLEEP)   {} ;              // controller in sleep mode
	if (ul_status & CAN_SR_WAKEUP)  {} ;              // controller woke up
	if (ul_status & CAN_SR_TOVF)    {} ;              // timer overflow
	if (ul_status & CAN_SR_TSTP)    {} ;              // timestamp - start or end of frame
	if (ul_status & CAN_SR_CERR)    {} ;              // CRC error in mailbox
	if (ul_status & CAN_SR_SERR)    numBusErrors++ ;  // stuffing error in mailbox
	if (ul_status & CAN_SR_AERR)    numBusErrors++ ;  // ack error
	if (ul_status & CAN_SR_FERR)    numBusErrors++ ;  // form error
	if (ul_status & CAN_SR_BERR)    numBusErrors++ ;  // bit error
}

//+=====================================================================================================================
// return: -1 .. nothing found ; >=0 .. free mailbox number
int  CANRaw::findFreeRXMailbox ()
{
	for (int  c = 0;  c < getNumMailBoxes();  c++)
		if ((mailbox_get_mode(c) == CAN_MB_RX_MODE) && (mailbox_get_id(c) == 0))  return c ;
	return -1;
}

//+=====================================================================================================================
/**
* \brief Set up an RX mailbox (first free) for the given parameters.
* \param id - the post mask ID to match against
* \param mask - the mask to use for this filter
* \param extended - whether to use 29 bit filter
* \ret number of mailbox we just used (or -1 if there are no free boxes to use)
*/
int  CANRaw::_setFilter (uint32_t id,  uint32_t mask,  bool extended)
{
	int  c = findFreeRXMailbox();

	if (c < 0)  return -1 ;

	mailbox_set_accept_mask(c, mask, extended);
	mailbox_set_id(c, id, extended);
	enable_interrupt(getMailboxIer(c));

	return c;
}

//+=============================================================================
/**
* \brief Set up an RX mailbox (given MB number) filter
* \param mailbox Which mailbox to use (0-7)
* \param id The ID to match against
* \param mask The mask to apply before ID matching
* \param extended Whether this should be extended mask or not
* \retval Mailbox number if successful or -1 on failure
*/
int  CANRaw::_setFilterSpecific (uint8_t mailbox,  uint32_t id,  uint32_t mask,  bool extended)
{
	if (mailbox >= getNumMailBoxes())  return -1 ;

	// !!! shouldn't we be checking if this is an Rx mailbox?

	mailbox_set_accept_mask(mailbox, mask, extended);
	mailbox_set_id(mailbox, id, extended);
	enable_interrupt(getMailboxIer(mailbox));

	return mailbox;
}

//+=====================================================================================================================
uint32_t  CANRaw::getMailboxIer (int8_t mailbox)
{
	switch (mailbox) {
		case 0  :  return CAN_IER_MB0;
		case 1  :  return CAN_IER_MB1;
		case 2  :  return CAN_IER_MB2;
		case 3  :  return CAN_IER_MB3;
		case 4  :  return CAN_IER_MB4;
		case 5  :  return CAN_IER_MB5;
		case 6  :  return CAN_IER_MB6;
		case 7  :  return CAN_IER_MB7;
		default :  return 0;
	}
	return 0;
}

//+=====================================================================================================================
// \param mb which mailbox generated this event
// \param ul_status The status register of the canbus hardware
//
void  CANRaw::mailbox_int_handler (uint8_t mb,  uint32_t /*ul_status*/)
{
	CAN_FRAME      tempFrame;
	bool           caughtFrame = false;
	CANListener*   thisListener;
	ringbuffer_t*  pRing;

	if (mb >= getNumMailBoxes())  mb = getNumMailBoxes() - 1 ;  // !!!

	if (m_pCan->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) { //mailbox signals it is ready
		switch(((m_pCan->CAN_MB[mb].CAN_MMR >> 24) & 7)) { //what sort of mailbox is it?
			case 1: //receive
			case 2: //receive w/ overwrite
			case 4: //consumer - technically still a receive buffer
				mailbox_read(mb, &tempFrame);
				numRxFrames++;

				// Do we have a mailbox specific callback?
				if (cbCANFrame[mb]) {
					caughtFrame = true;
					(*cbCANFrame[mb])(&tempFrame);

				// Do we have a general callback?
				} else if (cbGeneral) {
					caughtFrame = true;
					(*cbGeneral)(&tempFrame);

				// How about a Listener?
				} else {
					for (int  listenerPos = 0;  listenerPos < SIZE_LISTENERS;  listenerPos++) {
						if ((thisListener = listener[listenerPos])) {
							if (thisListener->isCallbackActive(mb))  {
								caughtFrame = true;
								thisListener->gotFrame(&tempFrame, mb);

							} else if (thisListener->isCallbackActive(8)) {  //global catch-all
								caughtFrame = true;
								thisListener->gotFrame(&tempFrame, -1);
							}
						}
					}
				}
				// if none of the callback types caught this frame then queue it in the buffer
				if (!caughtFrame)  addToRingBuffer(rxRing, tempFrame) ;
				break;

			case 3: //transmit
				pRing = usesGlobalTxRing(mb) ? &txRing : txRings[mb];
				if (removeFromRingBuffer(*pRing, tempFrame)) //if there is a frame in the queue to send
					writeTxRegisters(tempFrame,mb);
				else
					disable_interrupt(0x01 << mb);
				break;

			case 5: // producer - technically still a transmit buffer  // !!! should this be before case 3: ??
			break;
		}
	}
}

//+=====================================================================================================================
// Interrupt dispatchers - Never directly call these
// These two functions needed because interrupt handlers cannot be part of a class
 //
void CAN0_Handler(void)  {  Can0.interruptHandler();  }
void CAN1_Handler(void)  {  Can1.interruptHandler();  }
