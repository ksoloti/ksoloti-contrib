#pragma once

#include "hal/lib/streams/chprintf.h"

// syncronization
//
// 1. Currently based on events per channel, cannot use burst, non efficient gpio
// 2. Counting based, wait for all output channels to be set then fire event, good sync (read 333us behind), can use burst, efficient gpio
//
// Also look at async rather than sync as the CS is held open for a long time for some reason.
// Contextual burst should be used for ADC and DAC channels, need to start at first of each
// Lazy read/write of both gpio registers or based on config. 

// Set to 0 for event based on channels, 1 for counting based
#define COUNTING_BASED 1

// Set to 1 For Debug messages
#define DEBUG 0

// Set to 1 For Logic Anylyser
// GPIOA, 4 = SPI Error
// GPIOA, 5 = Process Output Channels (COUNTING)
// GPIOA, 6 = Process Input Channels (COUNTING)
// GPIOA, 7 = Process Single Channels 
#define ANALYSER 0

#if COUNTING_BASED
	#define PROCESS_EVENT                 EVENT_MASK(0)
	#define PROCESS_EVENT_OUTPUT          EVENT_MASK(1)
	#define PROCESS_EVENT_INPUT           EVENT_MASK(2)
	#define PROCESS_EVENT_EXIT            EVENT_MASK(20)
#endif
namespace MAX11300
{
typedef enum
{
	vrNoRange,
	vr0toP10,
	vrN5toP5,
	vrN10to0,
	vr0toP2_5		// Input only
} VoltageRange;

typedef enum
{
	as1,
	as2,
	as4,
	as8,
	as16,
	as32,
	as54,
	as128
} ADCSamples;

typedef enum
{
	ll3_3v,
	ll5v,
	ll10v
} LogicLevel;

typedef enum
{
	chHiZ,
	chGPI,
	chUnimplemented2,
	chGPO,
	chUnimplemented3,
	chDAC,
	chUnimplemented4,
	chADC
} ChannelType;

#define CHANNEL_COUNT 20

class ChannelInfo
{
private:
	ChannelType		m_channelType = chHiZ;
	VoltageRange  m_voltageRange = vrNoRange;
	ADCSamples		m_adcSamples = as1;
	uint16_t			m_uValue = 0;
	bool					m_bValue = false;
	LogicLevel		m_logicLevel = ll3_3v;
	bool					m_bInitialised = false;

	static char 								m_sStatusBuffer[64];
	static const char * const 	m_sChannelType[];
	static const char * const 	m_sVoltageRange[];
	static const char * const 	m_sSamples[];
	static const char * const 	m_sLogicLevel[];

public:
	bool SetChannelInfo(ChannelType channelType, VoltageRange voltageRange = vr0toP10, ADCSamples adcSamples = as1, uint16_t uValue = 0, LogicLevel logicLevel = ll3_3v)
	{
		// Can only change from hi Z or to hi Z
		bool bResult = false;
		if(channelType == chHiZ || (m_channelType == chHiZ))
		{
			m_channelType = channelType;
			m_voltageRange = voltageRange;
			m_adcSamples = adcSamples;
			m_uValue = uValue;
			m_logicLevel = logicLevel;
			m_bValue = false;
			m_bInitialised = false;

			bResult = true;
		}

		return(bResult);
	}

	char *GetStatusString(void)
	{
		if(IsAnalog())
		{
			chsnprintf(m_sStatusBuffer, sizeof(m_sStatusBuffer), "%s, %s, %s = %u", m_sChannelType[m_channelType], m_sVoltageRange[m_voltageRange], m_sSamples[m_adcSamples], m_uValue);

		}
		else if(IsDigital())
		{
			chsnprintf(m_sStatusBuffer, sizeof(m_sStatusBuffer), "%s, %s = %s", m_sChannelType[m_channelType], m_sLogicLevel[m_logicLevel], m_bValue ? "true" : "false");
		}
		else
			chsnprintf(m_sStatusBuffer, sizeof(m_sStatusBuffer), "%s", m_sChannelType[m_channelType]);

		return m_sStatusBuffer;
	}

	bool UsesDAC(void)
	{
		return ((m_channelType == chDAC) || (m_channelType == chGPO));
	}

	bool UsesADC(void)
	{
		return ((m_channelType == chADC) || (m_channelType == chGPI));
	}

	bool IsInitialised(void)
	{
		return m_bInitialised;
	}

	void SetInitialised(bool bInitialised)
	{
		m_bInitialised = bInitialised;
	}

	bool IsDigital(void)
	{
		return ((m_channelType == chGPI) || (m_channelType == chGPO));
	}

	bool IsAnalog(void)
	{
		return ((m_channelType == chDAC) || (m_channelType == chADC));
	}

	bool IsOutput(void)
	{
		return ((m_channelType == chDAC) || (m_channelType == chGPO));
	}

	bool IsInput(void)
	{
		return ((m_channelType == chADC) || (m_channelType == chGPI));
	}

	void SetDigitalValue(bool bValue)
	{
		m_bValue = bValue;
	}

	bool GetDigitalValue(void)
	{
		return m_bValue;
	}

	void SetAnalogValue(uint16_t uValue)
	{
		m_uValue = uValue;
	}

	uint16_t GetAnalogValue(void)
	{
		return m_uValue;
	}

	ChannelType GetChannelType(void)
	{
		return m_channelType;
	}

	VoltageRange GetVoltageRange(void)
	{
		return m_voltageRange;
	}

	ADCSamples GetSamples(void)
	{
		return m_adcSamples;
	}
};


WORKING_AREA(waThreadX, 1024*2);
msg_t ThreadX();
Thread *pProcessThread;

bool bConnected = false;
bool bInititalised = false;
bool bDACInitialised = false;
bool bADCInitialised = false;
bool bConfigurationNeeded = false;
bool bSPIError = false;

uint16_t uInChannelCount = 0;
uint16_t uOutChannelCount = 0;

uint16_t uInStartChannel = 20;
uint16_t uOutStartChannel = 20;

uint16_t uOutChannelsSet = 0;
uint16_t uInChannelsRead = 0;

bool bGpioOutUsed = false;
bool bGpioInUsed = false;

uint32_t uGpioOut = 0;
uint32_t uGpioIn = 0;


char ChannelInfo::m_sStatusBuffer[64];
const char * const ChannelInfo::m_sChannelType[] = { "HiZ", "GPI", "?", "GPO", "?", "DAC", "?", "ADC"};
const char * const ChannelInfo::m_sVoltageRange[] = { "?", "0V -> +10V", "-5V -> +5V", "-10V -> 0V", "0V -> 2.5V"};
const char * const ChannelInfo::m_sSamples[] = { "1 Sample", "2 Samples", "4 Samples", "8 Samples", "16 Samples", "32 Samples", "64 Samples", "128 Samples"};
const char * const ChannelInfo::m_sLogicLevel[] = { "3.3V", "5V", "10V"};

ChannelInfo channels[CHANNEL_COUNT];
#if BOARD_KSOLOTI_CORE_H743 || BOARD_KSOLOTI_CORE_F427
uint8_t txbuf[32] SPILINK_DMA_SECTION;
uint8_t rxbuf[32] SPILINK_DMA_SECTION;
#else
uint8_t txbuf[32] __attribute__ ((section (".sram2")));
uint8_t rxbuf[32] __attribute__ ((section (".sram2")));
#endif

void spi_error_cb(SPIDriver *spip) 
{
#if ANALYSER	
	palWritePad(GPIOA, 4, true);
#endif

  LogTextMessage("MAX11300: SPI Error");

  bSPIError = true;

	#if ANALYSER	
	palWritePad(GPIOA, 4, false);
#endif
}

#if BOARD_KSOLOTI_CORE_H743 || BOARD_KSOLOTI_CORE_F427
    // V21 chibios
    const SPIConfig spi3cfg = {
        .circular         = false,
        .slave            = false,
        .data_cb          = NULL,
        .error_cb         = spi_error_cb,
        .ssport           = GPIOA,
        .sspad            = 15U,
        .cr1              = SPI_CR1_BR_0,
        .cr2              = 0U
    } ;
#else
    const SPIConfig spi3cfg = {NULL, GPIOA, 15, 0   |(0<<3) };
#endif

/// SPI first byte when writing MAX11300 (7-bit address in bits 0x7E; LSB=0 for write)
#define MAX11300Addr_SPI_Write(RegAddr) ( (uint8_t)(RegAddr << 1)     )

/// SPI first byte when reading MAX11300 (7-bit address in bits 0x7E; LSB=1 for read)
#define MAX11300Addr_SPI_Read(RegAddr)  ( (uint8_t)((RegAddr << 1) | 1) )


#define PIXI_DAC_DATA   0x60
#define PIXI_ADC_DATA   0x40

#define PIXI_DEVICE_ID      0x00
#define PIXI_DEVICE_CTRL    0x10
#define BRST    						0x4000
#define THSHDN  						0x0080
#define RS_CANCEL   				0x1000

#define PIXI_TEMP_INT_HIGH_THRESHOLD     0x19

#define TMPCTLINT   0x0100
#define TMPCTLEXT1  0x0200
#define TMPCTLEXT2  0x0400

#define PIXI_PORT_CONFIG    0x20
#define FUNCID              0xF000
#define FUNCPRM_RANGE       0x0700

#define CH_MODE_GPI         0x01
#define CH_MODE_GPO         0x03
#define CH_MODE_DAC         0x05
#define CH_MODE_ADC         0x07

#define DACREF              0x0040
#define DACCTL              0x000C
#define ADCCTL              0x0003

#define GPI_DAT_15_0				0x0B
#define GPI_DAT_19_16				0x0C
#define GPO_DAT_15_0				0x0D
#define GPO_DAT_19_16				0x0E



bool SetAnalogOutputChannel(uint8_t uChannel, VoltageRange voltageRange)
{
	bool bResult = false;

	if(uChannel < CHANNEL_COUNT)
  {
		if(bResult = channels[uChannel].SetChannelInfo(chDAC, voltageRange))
		{
			if(uChannel < uOutStartChannel)
				uOutStartChannel = uChannel;

	    bConfigurationNeeded = true;
			uOutChannelCount++;
		}
  }
	
	return bResult;
}

uint16_t GetVoltageOutputLevel(LogicLevel logicLevel)
{
	uint16_t uVoltageLevel = 0;
	switch(logicLevel)
	{
		case ll3_3v: uVoltageLevel = 1351; break;
		case ll5v:   uVoltageLevel = 2047; break;
		case ll10v:  uVoltageLevel = 4095; break;
	}
	return uVoltageLevel;
}

uint16_t GetVoltageTriggerLevel(void)
{
	// As far as I can work out the maximum trigger is 2.5v which I am guessing is 4095
	// seems to work anyway!
	return 4095;
}


bool SetDigitalOutputChannel(uint8_t uChannel, LogicLevel logicLevel)
{
	bool bResult = false;

	if(uChannel < CHANNEL_COUNT)
	{
		uint16_t uVoltageLevel = GetVoltageOutputLevel(logicLevel);
		if(bResult = channels[uChannel].SetChannelInfo(chGPO, vr0toP10, as1, uVoltageLevel, logicLevel))
		{
	    bConfigurationNeeded = true;
			uOutChannelCount++;
		}
	}

	return bResult;
}

bool SetAnalogInputChannel(uint8_t uChannel, VoltageRange voltageRange, ADCSamples adcSamples = as1)
{
	bool bResult = false;

	if(uChannel < CHANNEL_COUNT)
  {
		if(bResult = channels[uChannel].SetChannelInfo(chADC, voltageRange, adcSamples))
		{
			if(uChannel < uInStartChannel)
				uInStartChannel = uChannel;

			bConfigurationNeeded = true;
			uInChannelCount ++;
		}
  }

	return bResult;
}

bool SetDigitalInputChannel(uint8_t uChannel)
{
	bool bResult = false;

	if(uChannel < CHANNEL_COUNT)
	{
  	uint16_t uVoltageLevel = GetVoltageTriggerLevel();

		if(bResult ==channels[uChannel].SetChannelInfo(chGPI, vr0toP10, as1, uVoltageLevel, ll3_3v))
		{
		  bConfigurationNeeded = true;
			uInChannelCount ++;
		}
	}

	return bResult;
}

#if COUNTING_BASED
void TriggerOutputProcessIfNeeded(void)
{
		uOutChannelsSet++;
		if(uOutChannelsSet >= uOutChannelCount)
		{
			uOutChannelsSet = 0;
		  chEvtSignal(pProcessThread, ((eventmask_t)PROCESS_EVENT_OUTPUT));
		}
}

void TriggerInputProcessIfNeeded(void)
{
		uInChannelsRead++;
		if(uInChannelsRead >= uInChannelCount)
		{
			uInChannelsRead = 0;
		  chEvtSignal(pProcessThread, ((eventmask_t)PROCESS_EVENT_INPUT));
		}
}
#else
void TriggerProcess(uint8_t uChannel)
{
  chEvtSignal(pProcessThread, ((eventmask_t)1)<<uChannel);
}
#endif


void SetDigitalValue(uint8_t uChannel, bool bValue)
{
	if(uChannel < CHANNEL_COUNT)
	{
		channels[uChannel].SetDigitalValue(bValue);
#if COUNTING_BASED
		if(bValue)
			uGpioOut |= (uint32_t)1<<uChannel;
		else
			uGpioOut &= ~(uint32_t)1<<uChannel;

		TriggerOutputProcessIfNeeded();
#else			
		TriggerProcess(uChannel);
#endif
	}
}

bool GetDigitalValue(uint8_t uChannel)
{
	bool bValue = false;
	if(uChannel < CHANNEL_COUNT)
	{
#if COUNTING_BASED
		bValue = (uGpioIn >> uChannel) & 1;
		TriggerInputProcessIfNeeded();
#else					
		bValue = channels[uChannel].GetDigitalValue();
		TriggerProcess(uChannel);
#endif
	}
	return bValue;
}

void SetAnalogValue(uint8_t uChannel, uint16_t uValue)
{
	if(uChannel < CHANNEL_COUNT)
	{
		channels[uChannel].SetAnalogValue(uValue);
#if COUNTING_BASED
		TriggerOutputProcessIfNeeded();
#else			
		TriggerProcess(uChannel);
#endif
	}
}

uint16_t GetAnalogValue(uint8_t uChannel)
{
	uint16_t uValue = 0;
	if(uChannel < CHANNEL_COUNT)
	{
		uValue = channels[uChannel].GetAnalogValue();
#if COUNTING_BASED
		TriggerInputProcessIfNeeded();
#else					
		TriggerProcess(uChannel);
#endif
	}
	return uValue;
}

bool SpiTransmit(uint16_t uSize)
{
  bSPIError = false;

  spiSelect(&SPID3);
  spiSend(&SPID3, uSize, txbuf);
  spiUnselect(&SPID3);

#if DEBUG
	if(bSPIError)
	{
		LogTextMessage("SpiTransmit bSPIError = %u", bSPIError);
		LogTextMessage("tx %x, %x, %x", txbuf[0],txbuf[1],txbuf[2]);
	}
#endif

	return !bSPIError;
}


bool SpiTransmitReceive(uint16_t uSize)
{
  bSPIError = false;

  spiSelect(&SPID3);
  spiExchange(&SPID3, uSize, txbuf, rxbuf);
  spiUnselect(&SPID3);

#if DEBUG
	if(bSPIError)
	{
		LogTextMessage("SpiTransmitReceive bSPIError = %u", bSPIError);
		LogTextMessage("r tx %x, %x, %x", txbuf[0],txbuf[1],txbuf[2]);
		LogTextMessage("r rx %x, %x, %x", rxbuf[0],rxbuf[1],rxbuf[2]);
	}
#endif

return !bSPIError;
}


bool ReadRegister(uint8_t uAddress, uint16_t *puValue)
{
	bool bResult = false;

  txbuf[0] = MAX11300Addr_SPI_Read(uAddress);
  txbuf[1] = 0xFF;
  txbuf[2] = 0xFF;
  
  rxbuf[0] = 0;
  rxbuf[1] = 0;
  rxbuf[2] = 0;
  
	bResult = SpiTransmitReceive(3);

	if(bResult)
	{
		uint16_t value = (rxbuf[1]<<8) | rxbuf[2];
		*puValue = value;
	}

	return bResult;
}

bool ReadDoubleRegister(uint8_t uAddress, uint32_t *puValue)
{
	bool bResult = false;

  txbuf[0] = MAX11300Addr_SPI_Read(uAddress);
  txbuf[1] = 0xFF;
  txbuf[2] = 0xFF;
  txbuf[3] = 0xFF;
  txbuf[4] = 0xFF;
  
  rxbuf[0] = 0;
  rxbuf[1] = 0;
  rxbuf[2] = 0;
  rxbuf[3] = 0;
  rxbuf[4] = 0;
  
	bResult = SpiTransmitReceive(5);

	if(bResult)
	{
		uint32_t value = (rxbuf[1]<<8) | rxbuf[2] | rxbuf[3]<<24 | rxbuf[4]<<16;
		*puValue = value;
	}

	return bResult;
}

bool WriteRegister(uint8_t uAddress, const uint16_t uValue)
{
	txbuf[0] = MAX11300Addr_SPI_Write(uAddress);
	txbuf[1] = (uValue) >> 8;
	txbuf[2] = (uValue) & 0xFF;

	bool bResult = SpiTransmit(3);

	return bResult;
}

bool WriteDoubleRegister(uint8_t uAddress, const uint32_t uValue)
{
	txbuf[0] = MAX11300Addr_SPI_Write(uAddress);
	txbuf[1] = (uValue >> 8) & 0xFF;
	txbuf[2] = uValue & 0xFF;
	txbuf[3] = (uValue >> 24) & 0xFF;
	txbuf[4] = (uValue >> 16) & 0xFF;

	bool bResult = SpiTransmit(5);

	return bResult;
}

bool ModifyRegister(uint8_t uAddress, uint16_t uMask, uint16_t uValue)
{
	bool bResult = false;

	uint16_t uRegValue;
	if(ReadRegister(uAddress, &uRegValue))
	{
		uRegValue = (uRegValue & ~uMask) | uValue;

		bResult = WriteRegister(uAddress, uRegValue);
	}

	return bResult;
}


bool Reset(void)
{
	uint16_t uReset = 0x8000;
	bool bResult = WriteRegister(PIXI_DEVICE_CTRL, uReset);
	chThdSleepMilliseconds(1);
	return bResult;
}

bool IsConnected(void)
{
	uint16_t uDeviceId = 0;

	bConnected = ReadRegister(PIXI_DEVICE_ID, &uDeviceId);

	if(bConnected)
		bConnected = (uDeviceId == 0x0424);

	return bConnected;
}


bool Initialise(void)
{
	if(!bInititalised)
	{
    // First setup SPI3
    palSetPadMode(GPIOA, 15, PAL_MODE_OUTPUT_PUSHPULL); // CS
    palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL); // SCK
    palSetPadMode(GPIOD, 6, PAL_MODE_OUTPUT_PUSHPULL); // MOSI

    palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(6)); // SCK
    palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(6)); // MISO
    palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(5)); // MOSI

    spiStart(&SPID3, &spi3cfg);

#if ANALYSER	
		// debug
		palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(GPIOA, 5, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL);
		palWritePad(GPIOA, 7, false);
		palWritePad(GPIOA, 6, false);
		palWritePad(GPIOA, 5, false);
		palWritePad(GPIOA, 4, false);
#endif

#if DEBUG		
		if(IsConnected())
			LogTextMessage("MAX11300: Connected");
		else
			LogTextMessage("MAX11300: Not Connected");
#endif

		Reset();
		
    // Now setup basics on MAX11300
		// We want thermal shutdown THSHDN and also contextual addressing BRST
		uint16_t uControlValue = BRST | THSHDN;
		if(WriteRegister(PIXI_DEVICE_CTRL, uControlValue))
		{
			chThdSleepMilliseconds(1);

			// enable internal temp sensor and disable series resistor cancelation
			if(ReadRegister( PIXI_DEVICE_CTRL, &uControlValue))
			{
				if(WriteRegister( PIXI_DEVICE_CTRL, uControlValue | !RS_CANCEL ))
				{
					// Set int temp hi threshold
					if(WriteRegister(PIXI_TEMP_INT_HIGH_THRESHOLD, 0x0230 ))    // 70 deg C in .125 steps
					{
						// Keep int temp lo threshold at 0 deg C, negative values need function to write a two's complement number.
						// enable internal and both external temp sensors
						if(ReadRegister( PIXI_DEVICE_CTRL, &uControlValue ))
						{
							bInititalised = WriteRegister( PIXI_DEVICE_CTRL, uControlValue | TMPCTLINT | TMPCTLEXT1 | TMPCTLEXT2 );
						}
					}
				}
			}
		}

    if(bInititalised)
    {
			uint16_t uControlValue;
			ReadRegister( PIXI_DEVICE_CTRL, &uControlValue);
	
      LogTextMessage("MAX11300: initialised. CR=%x", uControlValue);

      // Start thread
      pProcessThread = chThdCreateStatic(waThreadX, sizeof(waThreadX), LOWPRIO, (tfunc_t)ThreadX, nullptr);
    }
    else
      LogTextMessage("MAX11300: failed to initialise.");
	}
	return bInititalised;
}

bool ConfigDAC(void)
{
	if(!bDACInitialised)
	{
		uint16_t uControlValue;
		if(ReadRegister( PIXI_DEVICE_CTRL, &uControlValue))
		{
			if(WriteRegister( PIXI_DEVICE_CTRL, uControlValue | DACREF | !DACCTL ))
			{
				bDACInitialised = true;
				chThdSleepMilliseconds(1);
			}
		}
	}

	return bDACInitialised;
}

bool ConfigADC(void)
{
	if(!bADCInitialised)
	{
		uint16_t uControlValue;
		if(ReadRegister( PIXI_DEVICE_CTRL, &uControlValue))
		{
			if(WriteRegister( PIXI_DEVICE_CTRL, uControlValue | ADCCTL ))
			{
				bADCInitialised = true;
				chThdSleepMilliseconds(1);
			}
		}
	}

	return bADCInitialised;
}

bool ConfigChannel(uint8_t uChannel)
{
	bool bResult = false;
		
	if(uChannel < CHANNEL_COUNT)
	{
		ChannelInfo &channel = channels[uChannel];
		if(!channel.IsInitialised())
		{
			bResult = true;

			if(channel.IsDigital())
			{
				if(channel.IsInput())
					bGpioInUsed = true;
				else
					bGpioOutUsed = true;
			}

			// Initialise DAC and ADC if needed
			if(channel.UsesADC())
				bResult = ConfigADC();
			else if(channel.UsesDAC())
				bResult = ConfigDAC();

			// Set initial level, this will set logic level for digital channels
			if(bResult)
				bResult = WriteRegister( PIXI_DAC_DATA + uChannel, channel.GetAnalogValue());

			// Now configure the port
			if(bResult)
			{
				uint32_t uConfigReg = ( ( (channel.GetChannelType() << 12 ) & FUNCID ) | ( (channel.GetVoltageRange() << 8 ) & FUNCPRM_RANGE ) ) | (channel.GetSamples()<<5) ;
				bResult = WriteRegister( PIXI_PORT_CONFIG + uChannel, uConfigReg);
				chThdSleepMilliseconds(10);
			}

			channel.SetInitialised(bResult);
		}
	}

	return bResult;
}


bool WriteAnalog(uint8_t uChannel, uint16_t uValue)
{
	return WriteRegister(PIXI_DAC_DATA + uChannel, uValue);
}

bool ReadAnalog(uint8_t uChannel, uint16_t *puValue)
{
	return ReadRegister(PIXI_ADC_DATA + uChannel, puValue);
}

void ClearDigital(void)
{
	WriteRegister( GPO_DAT_15_0, 0);
	WriteRegister( GPO_DAT_19_16, 0);
	WriteRegister( GPI_DAT_15_0, 0);
	WriteRegister( GPI_DAT_19_16, 0);
}

bool WriteDigital(uint8_t uChannel, bool bValue)
{
	uint8_t uAddress = (uChannel > 15) ? GPO_DAT_19_16 : GPO_DAT_15_0;
	uint8_t uOffset  = (uChannel > 15) ? uChannel-15 : uChannel;

	return ModifyRegister(uAddress, 1 << uOffset, bValue << uOffset);
}

bool ReadDigital(uint8_t uChannel, bool *pbValue)
{
	uint8_t uAddress = (uChannel > 15) ? GPI_DAT_19_16 : GPI_DAT_15_0;
	uint8_t uOffset  = (uChannel > 15) ? uChannel-15 : uChannel;

	bool bResult = false;
	uint16_t uRegValue;
	if((bResult = ReadRegister(uAddress, &uRegValue)))
		*pbValue = ((uRegValue >> uOffset) & 1);

	return bResult;
}

#if COUNTING_BASED
bool ProcessOutputChannels(void)
{
#if ANALYSER	
	palWritePad(GPIOA, 5, true);
#endif

	// First gpio out
	bool bResult = false;
	
	if(bGpioOutUsed)
		bResult = WriteDoubleRegister(GPO_DAT_15_0, uGpioOut);

	// Now any analog outs, contextual burst
	if(uOutStartChannel < 20)
	{
		txbuf[0] = MAX11300Addr_SPI_Write(PIXI_DAC_DATA + uOutStartChannel);
		uint8_t uPos = 1;
		for(uint8_t uC = uOutStartChannel; uC < 20; uC++)
		{
			ChannelInfo &channel = channels[uC];
			if(channel.IsAnalog() && channel.IsOutput())
			{
				uint16_t uValue = channel.GetAnalogValue();
				txbuf[uPos++] = uValue >> 8;
				txbuf[uPos++] = uValue & 0xFF;
			}
		}
		SpiTransmit(uPos);
	}

#if ANALYSER	
	palWritePad(GPIOA, 5, false);
#endif

	return bResult;
}

bool ProcessInputChannels(void)
{
#if ANALYSER	
	palWritePad(GPIOA, 6, true);
#endif

	// First gpio in
	bool bResult = false;
	
	if(bGpioInUsed)
		ReadDoubleRegister(GPI_DAT_15_0, &uGpioIn);

	// Now any analog ins, contextual burst
	if(uInStartChannel < 20)
	{
		txbuf[0] = MAX11300Addr_SPI_Read(PIXI_ADC_DATA + uInStartChannel);
		uint8_t uPos = 1 + (uInChannelCount *2);
		bResult = SpiTransmitReceive(uPos);

		uPos = 1;
		for(uint8_t uC = uInStartChannel; uC < 20; uC++)
		{
			ChannelInfo &channel = channels[uC];
			if(channel.IsAnalog() && channel.IsInput())
			{
				uint16_t uValue = (rxbuf[uPos++]<<8) | rxbuf[uPos++];
				channel.SetAnalogValue(uValue);
			}
		}
	}

	#if ANALYSER	
	palWritePad(GPIOA, 6, false);
#endif

	return bResult;
}

#else // COUNTING_BASED

bool ProcessChannel(uint16_t uChannel)
{
	bool bResult = true;
	ChannelInfo &channel = channels[uChannel];

#if ANALYSER	
	palWritePad(GPIOA, 7, true);
#endif

	switch (channel.GetChannelType())
	{
		case chGPO:
		{
			bResult = WriteDigital(uChannel, channel.GetDigitalValue());
			break;
		}

		case chGPI:
		{
			bool bValue;
			if(ReadDigital(uChannel, &bValue))
			{
				channel.SetDigitalValue(bValue);
				bResult = true;
			}
			break;
		}

		case chDAC:
		{
			bResult = WriteAnalog(uChannel, channel.GetAnalogValue());
			break;
		}

		case chADC:
		{
			uint16_t uValue;
			if(ReadAnalog(uChannel, &uValue))
			{
				channel.SetAnalogValue(uValue);
				bResult = true;
			}
			break;
		}

		default: break;
	}

#if ANALYSER	
	palWritePad(GPIOA, 7, false);
#endif

	return bResult;
}
#endif // COUNTING_BASED

void DebugDump(void)
{
#if DEBUG
	for(uint8_t u=0; bInititalised && (u < CHANNEL_COUNT); u++)
	{
		if(channels[u].IsAnalog() || channels[u].IsDigital())
		{
			LogTextMessage("channel %u : %s\r\n", u, channels[u].GetStatusString());
		}
	}
#endif	
}

msg_t ThreadX()
{
#if DEBUG	
  LogTextMessage("MAX11300 thread started");
#endif
	ClearDigital();
  while ( !chThdShouldTerminate() ) 
  {
    eventmask_t evt = chEvtWaitOne((eventmask_t)ALL_EVENTS);
		//LogTextMessage("event= %x", evt);
		
    // do we need to config anything
    if(bConfigurationNeeded)
    {
      bConfigurationNeeded=false;

			for(uint8_t u=0; bInititalised && (u < CHANNEL_COUNT); u++)
			{
				bool bConfigured = ConfigChannel(u);
#if DEBUG				
				if(bConfigured)
					LogTextMessage("Initialised channel %u as %s", u, channels[u].GetStatusString());
				else
					LogTextMessage("Failed to initialise channel %u as %s", u, channels[u].GetStatusString());
#endif
			}
    }
#if COUNTING_BASED
		if(evt & PROCESS_EVENT_OUTPUT)
		{
			ProcessOutputChannels();
		}
		else if(evt & PROCESS_EVENT_INPUT)
		{
			ProcessInputChannels();
		}
		else if(evt & PROCESS_EVENT_EXIT)
			break;
#else	// COUNTING_BASED	
    // process the channels
		for(uint8_t uB =0; uB < 20; uB++)
		{
			if(evt&1)
    		ProcessChannel(uB);
			evt = evt>>1;
		}

		// 20th bit is exit
		if(evt&1)
			break;
#endif // COUNTING_BASED
 }

#if DEBUG 
  LogTextMessage("MAX11300 thread terminated");
#endif

  chThdExit((msg_t)0);
}

void Terminate(void)
{
	if(bInititalised)
	{
		static bool bTerminated = false;
		if(!bTerminated)
		{
			bTerminated = true;

			// Trigger process loop to exit
#if COUNTING_BASED
		  chEvtSignal(pProcessThread, ((eventmask_t)PROCESS_EVENT_EXIT));
#else	// COUNTING_BASED		
			TriggerProcess(20);
#endif // COUNTING_BASED
			chThdTerminate( pProcessThread );
			chThdWait( pProcessThread );
		}
	}
}

}; // namespace
