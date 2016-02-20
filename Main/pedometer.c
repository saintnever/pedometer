/**************************************************************************//**
 * @file
 * @brief pedometer demo for EFM32WG_STK3800 development kit
 ******************************************************************************
 *
 * This file is licensed under the Apache License Agreement. See the file
 * "LICENSE" for details.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "i2cspm.h"
#include "bsp.h"
#include "segmentlcd.h"
#include "bsp_trace.h"
#include "udelay.h"

/** Driver handle instances */
I2CSPM_Init_TypeDef ADXL345Handle = I2CSPM_INIT_DEFAULT;
#define	ADXL345I2CADDR   		0xA6	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
											//ALT  ADDRESS引脚接地时地址为0xA6，接电源时地址为0x3A
#define	DEVICEID_REG_ADDR		0x00
#define	DATA_START_REG_ADDR		0x32
#define	DATA_STOP_REG_ADDR		0x37
#define	FIFO_CTL_REG_ADDR		0x38
#define	FIFO_STA_REG_ADDR		0x39

#define ADXL345_I2C_SCL_PORT    gpioPortC
#define ADXL345_I2C_SCL_PIN     5
#define ADXL345_I2C_SDA_PORT    gpioPortC
#define ADXL345_I2C_SDA_PIN     4
#define ADXL345_I2C_PORT_LOC    0

//define ADXL345 data structure
typedef struct
{
  float X;
  float Y;
  float Z;
  
} SENSOR_DATA_TypeDef;

//Step count definitions
SENSOR_DATA_TypeDef THRESH			= {35,225,225};
SENSOR_DATA_TypeDef MAX_DATA 		= {0,0,0};              
SENSOR_DATA_TypeDef MIN_DATA 		= {1000,1000,1000};
SENSOR_DATA_TypeDef DELTA 			= {1000,1000,1000};                    //max-min to select most active axis
SENSOR_DATA_TypeDef RES             = {1000,1000,1000};
SENSOR_DATA_TypeDef LAST_SAMPLE 	= {0,0,0};
SENSOR_DATA_TypeDef CURR_SAMPLE 	= {0,0,0};	
SENSOR_DATA_TypeDef AVERAGE_DATA[4] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
int STEP_COUNT=0;

#define max(a,b) ((a) > (b) ? (a) : (b))

volatile uint32_t msTicks; /* counts 1ms timeTicks */

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256

volatile struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} rxBuf, txBuf = { {0}, 0, 0, 0, false };


/* Setup USART1 in async mode for RS232*/
static USART_TypeDef           * uart   = USART1;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;


static int32_t 				ADXL345_Read_Reg(I2C_TypeDef *i2c, uint8_t reg_addr, uint8_t *data, uint32_t len);
static int32_t 				ADXL345_Single_Write_Reg(I2C_TypeDef *i2c, uint8_t reg_addr, uint8_t data);
void 		   				ADXL345_INIT(I2C_TypeDef *i2c);
void 		   				ADXL345_CALIBRATE(I2C_TypeDef *i2c);
SENSOR_DATA_TypeDef		    ADXL345_DATA_CONVERT(uint16_t *data);
SENSOR_DATA_TypeDef 		ADXL345_ADD(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1);
SENSOR_DATA_TypeDef 		ADXL345_MINUS(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1);
SENSOR_DATA_TypeDef 		ADXL345_TIMES(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1);
SENSOR_DATA_TypeDef 		ADXL345_DEVIDE(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1);
SENSOR_DATA_TypeDef 		ADXL345_AVERAGE(SENSOR_DATA_TypeDef *data, uint32_t len);
bool 						ADXL345_IS_NULL(SENSOR_DATA_TypeDef data);
void 						ADXL345_STEPCOUNT(SENSOR_DATA_TypeDef NEW_SAMPLE);



/**************************************************************************//**
 * @brief
 *  Reads data from the ADXL345 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The register address of the sensor.
 * @param[out] data
 *   The data read from the sensor.
 * @param[in] len
 *   The length of data to be read back
 * @return
 *   Returns number of bytes read on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
static int32_t ADXL345_Read_Reg(I2C_TypeDef *i2c, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[len];
  uint8_t                    i2c_write_data[1];

  seq.addr  = ADXL345I2CADDR;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = reg_addr;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = len;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
  {
    *data = 0;
    return((int) ret);
  }
  
  // *data=i2c_read_data[0];
  for(uint32_t i=0;i<len;i++)
  {
	  data[i] = i2c_read_data[i];
  }
  // *data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);

  return((int) 2);
}

/**************************************************************************//**
 * @brief
 *  writes data to registers of ADXL345 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The register address of the sensor.
 * @param[out] data
 *   The data to write to the sensor.
 * @param[in] len
 *   The length of data to be read back
 * @return
 *   Returns number of bytes read on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
static int32_t ADXL345_Single_Write_Reg(I2C_TypeDef *i2c, uint8_t reg_addr, uint8_t data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[0];
  uint8_t                    i2c_write_data[2];

  seq.addr  = ADXL345I2CADDR;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = reg_addr;  
  i2c_write_data[1] = data;  
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }
  
  return((int) 2);
}

/**************************************************************************//**
 * @brief
 * initiates ADXL345 sensor
 *****************************************************************************/
void ADXL345_INIT(I2C_TypeDef *i2c)
{
   ADXL345_Single_Write_Reg(i2c,0x31,0x2B);   //测量范围,正负16g，13位模式,output low at interrupts
   // ADXL345_Single_Write_Reg(i2c,0x31,0x26);   //测量范围,正负8g，10位模式,output low at interrupts
   ADXL345_Single_Write_Reg(i2c,0x2C,0x08);   //速率设定为12.5 参考pdf13页
   ADXL345_Single_Write_Reg(i2c,0x2D,0x08);   //选择电源模式   参考pdf24页
   ADXL345_Single_Write_Reg(i2c,0x2E,0x00);   //disable all interrupts
   ADXL345_Single_Write_Reg(i2c,0x2F,0x00);   //map interrupts to INT1
   ADXL345_Single_Write_Reg(i2c,0x2E,0x02);   //使能 watermark 中断
   // ADXL345_Single_Write_Reg(i2c,0x38,0x5F);   //FIFO in FIFO mode
   ADXL345_Single_Write_Reg(i2c,0x38,0x9F);   //FIFO in streaming mode
   ADXL345_CALIBRATE(i2c);
}

/**************************************************************************//**
 * @brief
 * CONVERT TO ACUTUAL G VALUE
 *****************************************************************************/
SENSOR_DATA_TypeDef ADXL345_DATA_CONVERT(uint16_t *data)
{
   int8_t sign = 1;
   uint16_t temp = 0;
   uint32_t i = 0;
   SENSOR_DATA_TypeDef converted_data;
   for(i=0;i<3;i++)
   {	
	   sign = 1;
	   temp = data[i];
	   if((temp&0xF000)==0xF000)
	   {
			sign = -1;
			temp = -temp;
	   }
	   if(i==0)
	   {
		   converted_data.X = (float)(temp&0x0FFF)*3.9*sign+720;   //calibrate out x axis offset
	   }
	   else if(i==1)
	   {
		   converted_data.Y = (float)(temp&0x0FFF)*3.9*sign+620;   //calibrate out y axis offset
	   }
	   else if(i==2)
	   {
		   converted_data.Z = (float)(temp&0x0FFF)*3.9*sign-5000;  //calibrate out z axis offset
	   }
   }
   return converted_data;
}


/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
void ADXL345_CALIBRATE(I2C_TypeDef *i2c)
{
	// uint32_t i;
	// uint32_t j;
	// uint16_t axis_data[3];
	// uint16_t avg_data[3];
	// SENSOR_DATA_TypeDef offset;
	// int8_t  offset_X;
	// int8_t  offset_Y;
	// int8_t  offset_Z;
	
	// for(i=0;i<10;i++)
	// {
		// ADXL345_Read_Reg(ADXL345Handle.port, DATA_START_REG_ADDR,(uint8_t*)axis_data,6);
		// for(j=0;j<3;j++)
		// {
			// avg_data[j] =  (avg_data[j]+axis_data[j])>>1;  //average
		// }
	// }
	// offset = ADXL345_DATA_CONVERT(avg_data);
	// //device issue??? offset too large, cannot be compensated by register
	// offset_X = (uint8_t) (-offset.X/15.6);
	// offset_Y = (uint8_t) (-offset.Y/15.6);
	// offset_Z = (uint8_t) (-(offset.Z-1000)/15.6);
    ADXL345_Single_Write_Reg(i2c,0x1E,0);   //X OFFSET
    ADXL345_Single_Write_Reg(i2c,0x1F,0);   //Y OFFSET
    ADXL345_Single_Write_Reg(i2c,0x20,0);   //Z OFFSET
}

/**************************************************************************//**
 * @brief
 * Read full FIFO of ADXL345 sensor, average over 4 consecutive data points
 *****************************************************************************/
void ADXL345_READ_FIFO(SENSOR_DATA_TypeDef *axis_converted_avg)
{
   uint16_t axis_data[33][3];
   uint8_t init_flag=0;
   SENSOR_DATA_TypeDef axis_converted[33];
   uint8_t res_div = 10;
   
   //reinit MAX and MIN value for each FIFO read
   MAX_DATA.X = 0;
   MAX_DATA.Y = 0;
   MAX_DATA.Z = 0;
   MIN_DATA.X = 1000;
   MIN_DATA.Y = 1000;
   MIN_DATA.Z = 1000;
   //check if this is the very first data collection
   if(ADXL345_IS_NULL(AVERAGE_DATA[0]) && ADXL345_IS_NULL(AVERAGE_DATA[1]) && ADXL345_IS_NULL(AVERAGE_DATA[2]))
   {
	   init_flag = 1;
   }
   for(uint8_t i=0;i<33;i++)
	{
		ADXL345_Read_Reg(ADXL345Handle.port, DATA_START_REG_ADDR,(uint8_t*)axis_data[i],6);
		axis_converted[i] = ADXL345_DATA_CONVERT(axis_data[i]);	
		//Take care of first data collection issue
		if(i==0 && init_flag)
		{
			AVERAGE_DATA[0] = axis_converted[i];
			AVERAGE_DATA[1] = axis_converted[i];
			AVERAGE_DATA[2] = axis_converted[i];
			AVERAGE_DATA[3] = axis_converted[i];
		}
		//average process
		AVERAGE_DATA[3] = AVERAGE_DATA[2];
		AVERAGE_DATA[2] = AVERAGE_DATA[1];
		AVERAGE_DATA[1] = AVERAGE_DATA[0];
		AVERAGE_DATA[0] = axis_converted[i];
	    axis_converted_avg[i] = ADXL345_AVERAGE(AVERAGE_DATA,4);
		//find maximum and minimum value of 3 axis
		if(MAX_DATA.X<axis_converted_avg[i].X)  MAX_DATA.X=axis_converted_avg[i].X;
		if(MAX_DATA.Y<axis_converted_avg[i].Y)  MAX_DATA.Y=axis_converted_avg[i].Y;
		if(MAX_DATA.Z<axis_converted_avg[i].Z)  MAX_DATA.Z=axis_converted_avg[i].Z;
		if(MIN_DATA.X>axis_converted_avg[i].X)  MIN_DATA.X=axis_converted_avg[i].X;
		if(MIN_DATA.Y>axis_converted_avg[i].Y)  MIN_DATA.Y=axis_converted_avg[i].Y;
		if(MIN_DATA.Z>axis_converted_avg[i].Z)  MIN_DATA.Z=axis_converted_avg[i].Z;		
	}	
	
	//Calculate resolution for step counting 
	THRESH = ADXL345_AVERAGE(axis_converted_avg,33);
	DELTA =  ADXL345_MINUS(MAX_DATA,MIN_DATA);
	RES.X = max(MAX_DATA.X-THRESH.X, THRESH.X-MIN_DATA.X)/res_div;
	RES.Y = max(MAX_DATA.Y-THRESH.Y, THRESH.Y-MIN_DATA.Y)/res_div;
	RES.Z = max(MAX_DATA.Z-THRESH.Z, THRESH.Z-MIN_DATA.Z)/res_div;
	
}

/**************************************************************************//**
 * @brief
 * Read full FIFO of ADXL345 sensor, average over 4 consecutive data points
 *****************************************************************************/
void ADXL345_READ_FIFO_RAW(uint16_t **axis_data)
{
  
   for(uint8_t i=0;i<33;i++)
	{
		ADXL345_Read_Reg(ADXL345Handle.port, DATA_START_REG_ADDR,(uint8_t*)axis_data[i],6);
	}
	
}


/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
void ADXL345_STEPCOUNT(SENSOR_DATA_TypeDef NEW_SAMPLE)
{
	//@TODO add time window and step model rule 
	if(abs(DELTA.X)>=abs(DELTA.Y) && abs(DELTA.X)>=abs(DELTA.Z) )
	{
		if((LAST_SAMPLE.X-THRESH.X>RES.X) && (CURR_SAMPLE.X-THRESH.X<-RES.X))
		{
			STEP_COUNT++;
			LAST_SAMPLE = CURR_SAMPLE;
		}	
		else
		{
			LAST_SAMPLE = CURR_SAMPLE;
			CURR_SAMPLE = NEW_SAMPLE;
		}
	}
	
	else if(abs(DELTA.Y)>=abs(DELTA.X) && abs(DELTA.Y)>=abs(DELTA.Z) )
	{
		if((LAST_SAMPLE.Y-THRESH.Y>RES.Y) && (CURR_SAMPLE.Y-THRESH.Y<-RES.Y))
		{
			STEP_COUNT++;
			LAST_SAMPLE = CURR_SAMPLE;
		}	
		else
		{
			LAST_SAMPLE = CURR_SAMPLE;
			CURR_SAMPLE = NEW_SAMPLE;
		}
	}
	
	else if(abs(DELTA.Z)>=abs(DELTA.X) && abs(DELTA.Z)>=abs(DELTA.X) )
	{
		if((LAST_SAMPLE.Z-THRESH.Z>RES.Z) && (CURR_SAMPLE.Z-THRESH.Z<-RES.Z))
		{
			STEP_COUNT++;
			LAST_SAMPLE = CURR_SAMPLE;
		}	
		else
		{
			LAST_SAMPLE = CURR_SAMPLE;
			CURR_SAMPLE = NEW_SAMPLE;
		}
	}
	
}

// /**************************************************************************//**
 // * @brief
 // * Search for the most active axis 
 // *****************************************************************************/
// uint8_t ADXL345_S(SENSOR_DATA_TypeDef DELTA)
// {
	// if(abs(DELTA.X)>abs(DELTA.Y))
	// {
		// if(abs(DELTA.X)>abs(DELTA.Z))
		// {
			// return 
		// }
	// }

	
// }

/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
SENSOR_DATA_TypeDef ADXL345_ADD(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1)
{
	SENSOR_DATA_TypeDef result;
	result.X = data0.X + data1.X;
	result.Y = data0.Y + data1.Y;
	result.Z = data0.Z + data1.Z;
	return result;
}

/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
SENSOR_DATA_TypeDef ADXL345_MINUS(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1)
{
	SENSOR_DATA_TypeDef result;
	result.X = data0.X - data1.X;
	result.Y = data0.Y - data1.Y;
	result.Z = data0.Z - data1.Z;
	return result;
}

/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
SENSOR_DATA_TypeDef ADXL345_TIMES(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1)
{
	SENSOR_DATA_TypeDef result;
	result.X = data0.X * data1.X;
	result.Y = data0.Y * data1.Y;
	result.Z = data0.Z * data1.Z;
	return result;
}

/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
SENSOR_DATA_TypeDef ADXL345_DEVIDE(SENSOR_DATA_TypeDef data0, SENSOR_DATA_TypeDef data1)
{
	SENSOR_DATA_TypeDef result;
	result.X = data0.X/data1.X;
	result.Y = data0.Y/data1.Y;
	result.Z = data0.Z/data1.Z;
	return result;
}

/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
SENSOR_DATA_TypeDef ADXL345_AVERAGE(SENSOR_DATA_TypeDef *data, uint32_t len)
{
	SENSOR_DATA_TypeDef result;
	uint32_t i;
	for(i=0;i<len;i++)
	{
		result.X += data[i].X;
		result.Y += data[i].Y;
		result.Z += data[i].Z;
	}
	result.X /= len;
	result.Y /= len;
	result.Z /= len;
	return result;
}



/**************************************************************************//**
 * @brief
 * Calibrate ADXL345 sensor
 *****************************************************************************/
bool ADXL345_IS_NULL(SENSOR_DATA_TypeDef data)
{
	if(data.X==0 && data.Y==0 && data.Z==0)
	{
		return true;
	}
	else return false;
}


/**************************************************************************//**
 * @brief  Setup UART
 *****************************************************************************/
void setupUart(void)
{
/* Enable clock for GPIO module (required for pin configuration) */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* Configure GPIO pins */
  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);

  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = 115200;         /* Baud rate */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  uartInit.mvdis        = false;          /* Disable majority voting */
  uartInit.mvdis        = false;          /* Disable majority voting */
  uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */
  
  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, &uartInit);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, _UART_IFC_MASK);
  USART_IntEnable(uart, UART_IEN_RXDATAV);
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_ClearPendingIRQ(USART1_TX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_TX_IRQn);

  /* Enable I/O pins at USART1 location #2 */
  uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC1;

  /* Enable UART */
  USART_Enable(uart, usartEnable);
}

/**************************************************************************//**
 * @brief USART1 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  if (uart->IF & UART_IF_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(uart);
    rxBuf.data[rxBuf.wrI] = rxData;
    rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
    rxBuf.pendingBytes++;

    /* Flag Rx overflow */
    if (rxBuf.pendingBytes > BUFFERSIZE)
    {
      rxBuf.overflow = true;
    }
  }
}

/**************************************************************************//**
 * @brief USART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  /* Check TX buffer level status */
  if (uart->IF & UART_IF_TXBL)
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, txBuf.data[txBuf.rdI]);
      txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
      txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, UART_IEN_TXBL);
    }
  }
}

/******************************************************************************
 * @brief  uartPutData function
 *
 *****************************************************************************/
void uartPutData(uint8_t * dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;

  /* Check if buffer is large enough for data */
  if (dataLen > BUFFERSIZE)
  {
    /* Buffer can never fit the requested amount of data */
    return;
  }

  /* Check if buffer has room for new data */
  if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
  {
    /* Wait until room */
    while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) ;
  }

  /* Fill dataPtr[0:dataLen-1] into txBuffer */
  while (i < dataLen)
  {
    txBuf.data[txBuf.wrI] = *(dataPtr + i);
    txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;
    i++;
  }

  /* Increment pending byte counter */
  txBuf.pendingBytes += dataLen;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, UART_IEN_TXBL);
}

// ***************************************************************************//**
 // * @brief Set up Clock Management Unit
 // ******************************************************************************/
void cmuSetup(void)
{
  /* Start HFXO and wait until it is stable */
  /* CMU_OscillatorEnable( cmuOsc_HFXO, true, true); */

  /* Select HFXO as clock source for HFCLK */
  /* CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO ); */

  /* Disable HFRCO */
  /* CMU_OscillatorEnable( cmuOsc_HFRCO, false, false ); */

  /* Enable clock for HF peripherals */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Enable clock for USART module */
  CMU_ClockEnable(cmuClock_USART1, true);
}




/* Local prototypes */
void Delay(uint32_t dlyTicks);

/**************************************************************************//**
 * @brief SysTick_Handler
 *   Interrupt Service Routine for system tick counter
 * @note
 *   No wrap around protection
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}


/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  int i;
  uint8_t data[7];
  SENSOR_DATA_TypeDef axis_converted_avg[33];
  char lcd_data[20];
  /* ADXL345 I2C driver config */
  ADXL345Handle.port = I2C1;
  ADXL345Handle.sclPort = ADXL345_I2C_SCL_PORT;
  ADXL345Handle.sclPin = ADXL345_I2C_SCL_PIN;
  ADXL345Handle.sdaPort = ADXL345_I2C_SDA_PORT;
  ADXL345Handle.sdaPin = ADXL345_I2C_SDA_PIN;
  ADXL345Handle.portLocation = ADXL345_I2C_PORT_LOC;
  /* Chip errata */
  CHIP_Init();
  cmuSetup();
  /* If first word of user data page is non-zero, enable eA Profiler trace */
  // BSP_TraceProfilerSetup();

  // /* Enable two leds to show we're alive */
  // BSP_LedsInit();
  // BSP_LedSet(0);
  // BSP_LedSet(1);
  
  /* Configure UART operation */
  setupUart();

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  /* Enable LCD without voltage boost */
  SegmentLCD_Init(false);
  /* Initialize I2C drivers */
  I2CSPM_Init(&ADXL345Handle);

	ADXL345_INIT(ADXL345Handle.port);
	SegmentLCD_Write("ADXL345");
	Delay(500);
	/* Read Device ID */
	SegmentLCD_Write("DEV_ID");
	Delay(500);
	ADXL345_Read_Reg(ADXL345Handle.port, DEVICEID_REG_ADDR,data,1);
	SegmentLCD_LowerHex(data[0]);
	Delay(500);
	/* Read FIFO CTL */
	SegmentLCD_Write("AXISDATA");
	Delay(500);
	/* Infinite loop with test pattern. */
	uint8_t welcome[20] = "hello welcome";
	uartPutData(welcome,15);
	uint16_t axis_data[33][3];
   while(1)
   {
	   // axis_converted_avg.X =  0;
	   // axis_converted_avg.Y =  0;
	   // axis_converted_avg.Z =  0;
	   ADXL345_Read_Reg(ADXL345Handle.port, 0x30,data,1);
	   if(data[0]&0x02)
	   {   
		for(uint8_t i=0;i<33;i++)
			{
				ADXL345_Read_Reg(ADXL345Handle.port, DATA_START_REG_ADDR,(uint8_t*)axis_data[i],6);
				uartPutData((uint8_t*)axis_data[i],6);
				uartPutData("\n",sizeof("\n"));
			}
	   }
	  // SegmentLCD_LowerNumber(STEP_COUNT);
	// // SegmentLCD_LowerNumber(ADXL345_DATA_CONVERT(axis_data[0]));
	 // Delay(500);        
	// SegmentLCD_LowerNumber(ADXL345_DATA_CONVERT(axis_data[1]));
	// Delay(500);          
  }


}
