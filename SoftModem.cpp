#include "SoftModem.h"

#SoftModem code revised for ATTiny85
#ATTiny85 uses Timer0 as opposed to Timer2 used by ATMEGA328P and described in SoftModem github

#define TX_PIN  (3)
#define RX_PIN1 (0)  // AIN0 for ATTiny85

SoftModem *SoftModem::activeObject = 0;

SoftModem::SoftModem() {
}

SoftModem::~SoftModem() {
	end();
}

#if F_CPU == 16000000
#if SOFT_MODEM_BAUD_RATE <= 126
#define TIMER_CLOCK_SELECT	   (7)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(1024))
#elif SOFT_MODEM_BAUD_RATE <= 315
#define TIMER_CLOCK_SELECT	   (6)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(256))
#elif SOFT_MODEM_BAUD_RATE <= 630
#define TIMER_CLOCK_SELECT	   (5)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(128))
#elif SOFT_MODEM_BAUD_RATE <= 1225
#define TIMER_CLOCK_SELECT	   (4)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(64))
#else
#define TIMER_CLOCK_SELECT	   (3)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(32))
#endif
#else
#if SOFT_MODEM_BAUD_RATE <= 126
#define TIMER_CLOCK_SELECT	   (6)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(256))
#elif SOFT_MODEM_BAUD_RATE <= 315
#define TIMER_CLOCK_SELECT	   (5)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(128))
#elif SOFT_MODEM_BAUD_RATE <= 630
#define TIMER_CLOCK_SELECT	   (4)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(64))
#else
#define TIMER_CLOCK_SELECT	   (3)
#define MICROS_PER_TIMER_COUNT   (clockCyclesToMicroseconds(32))
#endif
#endif

#define BIT_PERIOD            (1000000/SOFT_MODEM_BAUD_RATE)
#define HIGH_FREQ_MICROS      (1000000/SOFT_MODEM_HIGH_FREQ)
#define LOW_FREQ_MICROS       (1000000/SOFT_MODEM_LOW_FREQ)

#define HIGH_FREQ_CNT         (BIT_PERIOD/HIGH_FREQ_MICROS)
#define LOW_FREQ_CNT          (BIT_PERIOD/LOW_FREQ_MICROS)

#define MAX_CARRIR_BITS	      (40000/BIT_PERIOD)

#define TCNT_BIT_PERIOD		  (BIT_PERIOD/MICROS_PER_TIMER_COUNT)
#define TCNT_HIGH_FREQ		  (HIGH_FREQ_MICROS/MICROS_PER_TIMER_COUNT)
#define TCNT_LOW_FREQ		  (LOW_FREQ_MICROS/MICROS_PER_TIMER_COUNT)

#define TCNT_HIGH_TH_L		  (TCNT_HIGH_FREQ * 0.90)
#define TCNT_HIGH_TH_H		  (TCNT_HIGH_FREQ * 1.15)
#define TCNT_LOW_TH_L		  (TCNT_LOW_FREQ * 0.85)
#define TCNT_LOW_TH_H		  (TCNT_LOW_FREQ * 1.10)

#if SOFT_MODEM_DEBUG_ENABLE
static volatile uint8_t *_portLEDReg;
static uint8_t _portLEDMask;
#endif

enum { START_BIT = 0, DATA_BIT = 8, STOP_BIT = 9, INACTIVE = 0xff };

void SoftModem::begin(void)
{
	pinMode(RX_PIN1, INPUT);
	digitalWrite(RX_PIN1, LOW);
	
	// pinMode(RX_PIN2, INPUT);
	// digitalWrite(RX_PIN2, LOW);
	
	pinMode(TX_PIN, OUTPUT);
	digitalWrite(TX_PIN, LOW);
	
	_txPortReg = portOutputRegister(digitalPinToPort(TX_PIN));
	_txPortMask = digitalPinToBitMask(TX_PIN);
	
#if SOFT_MODEM_DEBUG_ENABLE
	_portLEDReg = portOutputRegister(digitalPinToPort(13));
	_portLEDMask = digitalPinToBitMask(13);
	pinMode(13, OUTPUT);
#endif
	
	_recvStat = INACTIVE;
	_recvBufferHead = _recvBufferTail = 0;
	
	SoftModem::activeObject = this;
	
	_lastTCNT = TCNT0;
	_lastDiff = _lowCount = _highCount = 0;
	
	TCCR0A = 0;
	TCCR0B = TIMER_CLOCK_SELECT;
	ACSR   = _BV(ACIE) | _BV(ACIS1);
	DIDR0  = _BV(AIN1D) | _BV(AIN0D);
}

void SoftModem::end(void)
{
	ACSR   &= ~(_BV(ACIE));
	TIMSK &= ~(_BV(OCIE0A));
	DIDR0  &= ~(_BV(AIN1D) | _BV(AIN0D));
	SoftModem::activeObject = 0;
}

void SoftModem::demodulate(void)
{
	uint8_t t = TCNT0;
	uint8_t diff;
	
	diff = t - _lastTCNT;
	
	if(diff < 4)
		return;
	
	_lastTCNT = t;
	
	if(diff > (uint8_t)(TCNT_LOW_TH_H))
		return;
	
	// Calculating the moving average
#if SOFT_MODEM_MOVING_AVERAGE_ENABLE
	_lastDiff = (diff >> 1) + (diff >> 2) + (_lastDiff >> 2);
#else
	_lastDiff = diff;
#endif

	if(_lastDiff >= (uint8_t)(TCNT_LOW_TH_L)){
		_lowCount += _lastDiff;
		if(_recvStat == INACTIVE){
			// Start bit detection
			if(_lowCount >= (uint8_t)(TCNT_BIT_PERIOD * 0.5)){
				_recvStat = START_BIT;
				_highCount = 0;
				_recvBits  = 0;
				OCR0A = t + (uint8_t)(TCNT_BIT_PERIOD) - _lowCount;
				TIFR |= _BV(OCF0A);
				TIMSK |= _BV(OCIE0A);
			}
		}
	}
	else if(_lastDiff <= (uint8_t)(TCNT_HIGH_TH_H)){
		if(_recvStat == INACTIVE){
			_lowCount = 0;
			_highCount = 0;
		}
		else{
			_highCount += _lastDiff;
		}
	}
}

// Analog comparator interrupt
ISR(ANALOG_COMP_vect)
{
	SoftModem::activeObject->demodulate();
}

void SoftModem::recv(void)
{
	uint8_t high;
	
	// Bit logic determination
	if(_highCount > _lowCount){
		_highCount = 0;
		high = 0x80;
	}
	else{
		_lowCount = 0;
		high = 0x00;
	}
	
	// Start bit reception
	if(_recvStat == START_BIT){
		if(!high){
			_recvStat++;
		}
		else{
			goto end_recv;
		}
	}
	// Data bit reception
	else if(_recvStat <= DATA_BIT) {
		_recvBits >>= 1;
		_recvBits |= high;
		_recvStat++;
	}
	// Stop bit reception
	else if(_recvStat == STOP_BIT){
		if(high){
			// Stored in the receive buffer
			uint8_t new_tail = (_recvBufferTail + 1) & (SOFT_MODEM_RX_BUF_SIZE - 1);
			if(new_tail != _recvBufferHead){
				_recvBuffer[_recvBufferTail] = _recvBits;
				_recvBufferTail = new_tail;
			}
			else{
				;// Overrun error detection
			}
		}
		else{
			;// Fleming error detection
		}
		goto end_recv;
	}
	else{
	end_recv:
		_recvStat = INACTIVE;
		TIMSK &= ~_BV(OCIE0A);
	}
}

// Timer 2 compare match interrupt A
ISR(TIMER0_COMPA_vect)
{
	OCR0A += (uint8_t)TCNT_BIT_PERIOD;
	SoftModem::activeObject->recv();
#if SOFT_MODEM_DEBUG_ENABLE
	*_portLEDReg ^= _portLEDMask;
#endif
}

int SoftModem::available()
{
	return (_recvBufferTail + SOFT_MODEM_RX_BUF_SIZE - _recvBufferHead) & (SOFT_MODEM_RX_BUF_SIZE - 1);
}

int SoftModem::read()
{
	if(_recvBufferHead == _recvBufferTail)
		return -1;
	int d = _recvBuffer[_recvBufferHead];
	_recvBufferHead = (_recvBufferHead + 1) & (SOFT_MODEM_RX_BUF_SIZE - 1);
	return d;
}

int SoftModem::peek()
{
	if(_recvBufferHead == _recvBufferTail)
		return -1;
	return _recvBuffer[_recvBufferHead];
}

void SoftModem::flush()
{
}

void SoftModem::modulate(uint8_t b)
{
	uint8_t cnt,tcnt,tcnt0;
	if(b){
		cnt = (uint8_t)(HIGH_FREQ_CNT);
		TCNT0 = (uint8_t)(TCNT_HIGH_FREQ / 2);
		tcnt = (uint8_t)(TCNT_HIGH_FREQ) - TCNT0;
	}else{
		cnt = (uint8_t)(LOW_FREQ_CNT);
		TCNT0 = (uint8_t)(TCNT_LOW_FREQ / 2);
		tcnt = (uint8_t)(TCNT_LOW_FREQ) - TCNT0;
	}
	do {
		cnt--;
		{
			OCR0B += tcnt;
			TIFR |= _BV(OCF0B);
			while(!(TIFR & _BV(OCF0B)));
		}
		*_txPortReg ^= _txPortMask;
		{
			OCR0B += TCNT0;
			TIFR |= _BV(OCF0B);
			while(!(TIFR & _BV(OCF0B)));
		}
		*_txPortReg ^= _txPortMask;
	} while (cnt);
}

//  Preamble bit before transmission
//  1 start bit (LOW)
//  8 data bits, LSB first
//  1 stop bit (HIGH)
//  ...
//  Postamble bit after transmission

size_t SoftModem::write(const uint8_t *buffer, size_t size)
{
	// To calculate the preamble bit length
	uint8_t cnt = ((micros() - _lastWriteTime) / BIT_PERIOD) + 1;
	if(cnt > MAX_CARRIR_BITS){
		cnt = MAX_CARRIR_BITS;
	}
	// Preamble bit transmission
	for(uint8_t i = 0; i<cnt; i++){
		modulate(HIGH);
	}
	size_t n = size;
	while (size--) {
		uint8_t data = *buffer++;
		// Start bit transmission
		modulate(LOW);
		// Data bit transmission
		for(uint8_t mask = 1; mask; mask <<= 1){
			if(data & mask){
				modulate(HIGH);
			}
			else{
				modulate(LOW);
			}
		}
		// Stop bit transmission
		modulate(HIGH);
	}
	// Postamble bit transmission
	modulate(HIGH);
	_lastWriteTime = micros();
	return n;
}

size_t SoftModem::write(uint8_t data)
{
	return write(&data, 1);
}
