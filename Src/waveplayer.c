/**
 ******************************************************************************
 * @file    waveplayer.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    2019
 * @brief   This file includes a HAL version of the Wave Player driver
 * for the STM32072B-EVAL demonstration.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ff.h"
#include <math.h>

#include "btypes.h"               // U_LONG definition
#include "ff.h"                   // f_open(), f_read(), f_close() declarations
#include "time_f.h"               // time_*_ms() declarations
#include "waveplayer.h"           // T_WAVE_CHUNK definition
#include <string.h>               // memset() declaration

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;

//extern TIM_HandleTypeDef htim6;

/** @defgroup WAVEPLAYER_Private_Variables
 * @{
 */
//#define AUDIO_BUFFER_SIZE     4080  /* max. 0xFFFF */ // too large for a SPI SDcard reader
#define AUDIO_BUFFER_SIZE     1024  /* max. 0xFFFF */

/**-----------------------------------------------------------------
  @brief      helper union for audio data buffer
------------------------------------------------------------------*/
typedef union
{
  uint32_t                    u32[2*AUDIO_BUFFER_SIZE / 4];
  uint16_t                    u16[2*AUDIO_BUFFER_SIZE / 2];
  uint8_t                     u8[2*AUDIO_BUFFER_SIZE];
} U_AUDIO_BUF;

/**-----------------------------------------------------------------
  @brief      const value defines for audio buffer state
------------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL
} E_AUDIO_BUF_STATE;

U_AUDIO_BUF                   AudioBuf;       ///< buffer used for playing audio
volatile E_AUDIO_BUF_STATE    bufferOffset;   ///< Position in the audio play buffer
FIL                           fileObj;        ///< File object, contains buffers; size is > 2 KB
float                         volume = 0.05f;  ///< volume for audio file data post process
uint32_t                      fadeTimeRef;    ///< time reference for fade out
uint32_t                      fadeSubTime;    ///< sub time reference for fade out

/**-----------------------------------------------------------------
  @short      Volume control for 2 channel 16 bit sampled wave data
  @remark     tone control may be added here
  @param[out] buf ... start address of buffer to be adjusted
  @param[in]  num ... number of words to be adjusted
------------------------------------------------------------------*/
static inline void adjust_16bit_data ( int16_t* buf, uint16_t num )
{
  float val;

  do
  {
    val    = *buf;
    val   *= volume;
    *buf++ = val;
  } while (--num );
}

/**-----------------------------------------------------------------
  @short      Volume control for 2 channel 24 bit sampled wave data
  @remark     converts 3 to 4 byte values
  @remark     tone control may be added here
  @param[out] buf ... start address of buffer to be adjusted
  @param[in]  num3 .. number of 3 byte blocks to be adjusted
  @param[in]  num4 .. number of 4 byte blocks to be adjusted
------------------------------------------------------------------*/
static inline void adjust_24bit_data ( const uint8_t* const buf,
                                       const uint16_t num3, uint16_t num4 )
{
  float           val;
  U_LONG          value;
  const uint8_t*  src;
  int32_t*        dest;

  src  = buf + num3 - 1;
  dest = (int32_t*)buf + num4 - 1;
  value.u8[0] = 0;

  do
  {
    value.u8[3] = *src--;
    value.u8[2] = *src--;
    value.u8[1] = *src--;

    val     = value.s32;
    val    *= volume;
    *dest-- = val;
  } while (--num4 );
}

/**-----------------------------------------------------------------
  @short      Volume control for 2 channel 32 bit sampled wave data
  @remark     tone control may be added here
  @param[out] buf ... start address of buffer to be adjusted
  @param[in]  num ... number of words to be adjusted
------------------------------------------------------------------*/
static inline void adjust_32bit_data ( int32_t* buf, uint16_t num )
{
  float val;

  do
  {
    val    = *buf;
    val   *= volume;
    *buf++ = val;
  } while (--num );
}

/**-----------------------------------------------------------------
  @short      Volume control for 1 channel 16 bit sampled wave data
  @remark     tone control may be added here
  @param[out] buf ... start address of buffer to be adjusted
  @param[in]  num ... number of words to be adjusted
------------------------------------------------------------------*/
static inline void adjust_16bit_mono_data ( int16_t* const buf, uint16_t num )
{
  float     val;
  int16_t*  src;
  int16_t*  dest;

  src  = buf + num - 1;
  dest = src + num;

  do
  {
    val     = *src--;
    val    *= volume;
    *dest-- = val;
    *dest-- = val;
  } while (--num );
}

/**-----------------------------------------------------------------
  @short      Volume control for 1 channel 24 bit sampled wave data
  @remark     converts 3 to 4 byte values
  @remark     tone control may be added here
  @param[out] buf ... start address of buffer to be adjusted
  @param[in]  num3 .. number of 3 byte blocks to be adjusted
  @param[in]  num4 .. number of 4 byte blocks to be adjusted
------------------------------------------------------------------*/
static inline void adjust_24bit_mono_data ( const uint8_t* const buf,
                                            const uint16_t num3, uint16_t num4 )
{
  float           val;
  U_LONG          value;
  const uint8_t*  src;
  int32_t*        dest;

  src  = buf + num3 - 1;
  dest = (int32_t*)buf + num4 - 1;
  value.u8[0] = 0;
  num4 >>= 1;

  do
  {
    value.u8[3] = *src--;
    value.u8[2] = *src--;
    value.u8[1] = *src--;

    val     = value.s32;
    val    *= volume;
    *dest-- = val;
    *dest-- = val;
  } while (--num4 );
}

/**-----------------------------------------------------------------
  @short      Volume control for 1 channel 32 bit sampled wave data
  @remark     tone control may be added here
  @param[out] buf ... start address of buffer to be adjusted
  @param[in]  num ... number of words to be adjusted
------------------------------------------------------------------*/
static inline void adjust_32bit_mono_data ( int32_t* const buf, uint16_t num )
{
  float     val;
  int32_t*  src;
  int32_t*  dest;

  src  = buf + num - 1;
  dest = src + num;

  do
  {
    val     = *src--;
    val    *= volume;
    *dest-- = val;
    *dest-- = val;
  } while (--num );
}

/**-----------------------------------------------------------------
  @short      Set volume
  @param[in]  vol ... volume to be set\n
                  0 = silent\n
               1000 = normal\n
              65535 = maximum
------------------------------------------------------------------*/
void set_volume ( const uint16_t vol )
{
  volume  = vol;
  volume /= 1000.0f;
}

/**-----------------------------------------------------------------
  @short      Fill audio buffer with data from file
  @param[out] bufStart ... start address of buffer to be set
  @param[in]  num ........ number of bytes to be set
  @param[in]  waveFormat . format of the wave file to be played
  @pre        bufStart is 4 byte aligned,\n
              needed for adjust_audio_data()
------------------------------------------------------------------*/
static inline bool read_audio_data ( uint8_t* bufStart, const uint16_t num,
                                     const T_WAVE_CHUNK* const waveFormat )
{
  bool      report;
  UINT      bytesRead;
  uint16_t  bytesToRead;

  if ( waveFormat->BitPerSample == 24 )
  { // read only 3/4 count of bytes
    bytesToRead = 3 * num;
    bytesToRead >>= 2;
  }
  else
  {
    bytesToRead = num;
  }

  if ( waveFormat->channels == 1 )
  {
    bytesToRead >>= 1;
  }

  report = f_read ( &fileObj, bufStart, bytesToRead, &bytesRead );
  if ( report == FR_OK  &&  bytesRead )
  {
    if ( bytesRead < bytesToRead )
    {
      memset ( bufStart + bytesRead, 0, bytesToRead - bytesRead );
    }

    if ( waveFormat->channels == 2 )
    {
      switch ( waveFormat->BitPerSample )
      {
      case 24:
        adjust_24bit_data ( bufStart, bytesToRead, num >> 2 );
        break;

      case 32:
        adjust_32bit_data ( (int32_t*)bufStart, bytesToRead >> 2 );
        break;

      case 16:
      default:
        adjust_16bit_data ( (int16_t*)bufStart, bytesToRead >> 1 );
        break;
      }
    }
    else  // 1 channel = Mono
    {
      switch ( waveFormat->BitPerSample )
      {
      case 24:
        adjust_24bit_mono_data ( bufStart, bytesToRead, num >> 2 );
        break;

      case 32:
        adjust_32bit_mono_data ( (int32_t*)bufStart, bytesToRead >> 2 );
        break;

      case 16:
      default:
        adjust_16bit_mono_data ( (int16_t*)bufStart, bytesToRead >> 1 );
        break;
      }
    }

    report = true;
  }
  else
  {
    report = false;
  }

  return report;
}

/**
 * @brief  Wave player Initialization
 * @param  None
 * @retval None
 */
void WavePlayer_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	  /* USER CODE END SAI1_Init 1 */
	  hsai_BlockA1.Instance = SAI1_Block_A;
	  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
	  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
	  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
	  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
	  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
	  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
	  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
	  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
	  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
	  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	  hsai_BlockA1.FrameInit.FrameLength = 32;
	  hsai_BlockA1.FrameInit.ActiveFrameLength = 16;
	  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
	  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	  hsai_BlockA1.SlotInit.SlotNumber = 2;
	  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
	  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
	  {
	    Error_Handler();
	  }

		__HAL_UNLOCK(&hsai_BlockA1);     // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
		__HAL_SAI_ENABLE(&hsai_BlockA1); // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
}

/**
 * @brief  Reads a number of bytes from the SPI Flash and reorder them in Big
 *         or little endian.
 * @param  NbrOfBytes: number of bytes to read.
 *         This parameter must be a number between 1 and 4.
 * @param  ReadAddr: external memory address to read from.
 * @param  Endians: specifies the bytes endianness.
 *         This parameter can be one of the following values:
 *             - LittleEndian
 *             - BigEndian
 * @retval Bytes read from the SPI Flash.
 */
static uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat)
{
	uint32_t index = 0;
	uint32_t temp = 0;

	for (index = 0; index < NbrOfBytes; index++)
	{
		temp |= buffer[idx + index] << (index * 8);
	}

	if(BytesFormat == BigEndian)
	{
		temp = __REV(temp);
	}
	return temp;
}


/**
 * @brief  Checks the format of the .WAV file and gets information about the audio
 *         format. This is done by reading the value of a number of parameters
 *         stored in the file header and comparing these to the values expected
 *         authenticates the format of a standard .WAV  file (44 bytes will be read).
 *         If  it is a valid .WAV file format, it continues reading the header
 *         to determine the  audio format such as the sample rate and the sampled
 *         data size. If the audio format is supported by this application, it
 *         retrieves the audio format in WAVE_Format structure and returns a zero
 *         value. Otherwise the function fails and the return value is nonzero.
 *         In this case, the return value specifies the cause of  the function
 *         fails. The error codes that can be returned by this function are declared
 *         in the header file.
 * @param  WavName: wav file name
 * @param  FileLen: wav file length
 * @retval Zero value if the function succeed, otherwise it returns a nonzero
 *         value which specifies the error code.
 */
static ErrorCode WavePlayer_WaveParsing(const char* WavName)
{
	uint32_t temp = 0x00;
	uint32_t extraformatbytes = 0;

	if(FATFS_LinkDriver(&SD_Driver, SD_Path) != FR_OK)
	{
		Error_Handler();
	}

	if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) != FR_OK)
	{
		/* FatFs Initialization Error */
		Error_Handler();
	}

	f_close(&fileObj);

	if(f_open(&fileObj, WavName, FA_READ) != FR_OK)
	{
		/* file Open for write Error */
		Error_Handler();
	}

	FRESULT res = f_read(&fileObj, AudioBuf.u8, AUDIO_BUFFER_SIZE, &BytesRead);
	if((BytesRead == 0) || (res != FR_OK)) /* EOF or Error */
	{
		/* file Read or EOF Error */
		Error_Handler();
	}

	/* Read chunkID, must be 'RIFF'  -------------------------------------------*/
	temp = ReadUnit(AudioBuf.u8, 0, 4, BigEndian);
	if(temp != CHUNK_ID)
	{
		return(Unvalid_RIFF_ID);
	}

	/* Read the file length ----------------------------------------------------*/
	WAVE_Format.RIFFchunksize = ReadUnit(AudioBuf.u8, 4, 4, LittleEndian);

	/* Read the file format, must be 'WAVE' ------------------------------------*/
	temp = ReadUnit(AudioBuf.u8, 8, 4, BigEndian);
	if(temp != FILE_FORMAT)
	{
		return(Unvalid_WAVE_Format);
	}

	/* Read the format chunk, must be'fmt ' ------------------------------------*/
	temp = ReadUnit(AudioBuf.u8, 12, 4, BigEndian);
	if(temp != FORMAT_ID)
	{
		return(Unvalid_FormatChunk_ID);
	}
	/* Read the length of the 'fmt' data, must be 0x10 -------------------------*/
	temp = ReadUnit(AudioBuf.u8, 16, 4, LittleEndian);
	if(temp != 0x10)
	{
		extraformatbytes = 1;
	}
	/* Read the audio format, must be 0x01 (PCM) -------------------------------*/
	WAVE_Format.FormatTag = ReadUnit(AudioBuf.u8, 20, 2, LittleEndian);
	if(WAVE_Format.FormatTag != WAVE_FORMAT_PCM)
	{
		return(Unsupported_FormatTag);
	}

	/* Read the number of channels, must be 0x02 (Stereo) ----------------------*/
	WAVE_Format.NumChannels = ReadUnit(AudioBuf.u8, 22, 2, LittleEndian);

	if(WAVE_Format.NumChannels != CHANNEL_STEREO)
	{
		return(Unsupported_Number_Of_Channel);
	}

	/* Read the Sample Rate ----------------------------------------------------*/
	WAVE_Format.SampleRate = ReadUnit(AudioBuf.u8, 24, 4, LittleEndian);
	/* Update the OCA value according to the .WAV file Sample Rate */
	switch (WAVE_Format.SampleRate)
	{
	case SAMPLE_RATE_8000 :
		TIM6ARRValue = 10000; // 6000;
		break; /* 8KHz = 80MHz / 10000 */
	case SAMPLE_RATE_16000 :
		TIM6ARRValue = 3000;
		break; /* 8KHz = 48MHz / 6000 */
	case SAMPLE_RATE_11025:
		TIM6ARRValue = 4353;
		break; /* 11.025KHz = 48MHz / 4353 */
	case SAMPLE_RATE_22050:
		TIM6ARRValue = 2176;
		break; /* 22.05KHz = 48MHz / 2176 */
	case SAMPLE_RATE_32000:
		TIM6ARRValue = 1500;
		break; /* 32KHz = 48MHz / 1500 */
	case SAMPLE_RATE_44100:
		TIM6ARRValue = 1814; // 1088;
		break; /* 44.1KHz = 80MHz / 1814 */

	default:
		return(Unsupported_Sample_Rate);
	}

	/* Read the Byte Rate ------------------------------------------------------*/
	WAVE_Format.ByteRate = ReadUnit(AudioBuf.u8, 28, 4, LittleEndian);

	/* Read the block alignment ------------------------------------------------*/
	WAVE_Format.BlockAlign = ReadUnit(AudioBuf.u8, 32, 2, LittleEndian);

	/* Read the number of bits per sample --------------------------------------*/
	WAVE_Format.BitsPerSample = ReadUnit(AudioBuf.u8, 34, 2, LittleEndian);

	if(WAVE_Format.BitsPerSample != BITS_PER_SAMPLE_16)
	{
		return(Unsupported_Bits_Per_Sample);
	}
	SpeechDataOffset = 36;
	/* If there is Extra format bytes, these bytes will be defined in "Fact Chunk" */
	if(extraformatbytes == 1)
	{
		/* Read th Extra format bytes, must be 0x00 ------------------------------*/
		temp = ReadUnit(AudioBuf.u8, 36, 2, LittleEndian);
		if(temp != 0x00)
		{
			return(Unsupported_ExtraFormatBytes);
		}
		/* Read the Fact chunk, must be 'fact' -----------------------------------*/
		temp = ReadUnit(AudioBuf.u8, 38, 4, BigEndian);
		if(temp != FACT_ID)
		{
			return(Unvalid_FactChunk_ID);
		}
		/* Read Fact chunk data Size ---------------------------------------------*/
		temp = ReadUnit(AudioBuf.u8, 42, 4, LittleEndian);

		SpeechDataOffset += 10 + temp;
	}
	/* Read the Data chunk, must be 'data' -------------------------------------*/
	temp = ReadUnit(AudioBuf.u8, SpeechDataOffset, 4, BigEndian);
	SpeechDataOffset += 4;
	if(temp != DATA_ID)
	{
		return(Unvalid_DataChunk_ID);
	}

	/* Read the number of sample data ------------------------------------------*/
	WAVE_Format.DataSize = ReadUnit(AudioBuf.u8, SpeechDataOffset, 4, LittleEndian);
	SpeechDataOffset += 4;
	//  WaveCounter =  SpeechDataOffset;
	return(Valid_WAVE_File);
}

/**-----------------------------------------------------------------
  @short      Play wave file from a mass storage
  @param[in]  fileName ... wave file name
  @remark     called from play_sound()
------------------------------------------------------------------*/
void wave_play_back ( const char* const fileName )
{
  UINT          bytesRead;
  T_WAVE_CHUNK  waveFormat;
  bool          run;

  // Open the Wave file to be played
  if ( f_open(&fileObj, fileName, FA_READ) == FR_OK )
  {
    // Read sizeof(WaveFormat) bytes from file
    f_read ( &fileObj, &waveFormat, sizeof(waveFormat), &bytesRead );
    if ( sizeof(waveFormat) == bytesRead )
    {
      // Initialize wave player: I2S Codec, DMA, TWI, IOExpander and IOs
      WavePlayer_Init();
      run = true;
    }
    else
    {
      run = false;
    }

// AJOUT JCT
#define WAV_FILE_HEADER_SIZE 44
if (f_lseek(&fileObj, WAV_FILE_HEADER_SIZE) != FR_OK)
{
  Error_Handler();
}

    if ( run )
    {
      // clear buffer for startup sequence
      memset ( AudioBuf.u8, 0, AUDIO_BUFFER_SIZE );

      bufferOffset = BUFFER_OFFSET_NONE;
      fadeTimeRef  = 0;
      fadeSubTime  = 0;

      // Start playing Wave of AUDIO_BUFFER_SIZE/2 words
      if ( waveFormat.BitPerSample == 16 )
      {
        HAL_SAI_Transmit_DMA ( &hsai_BlockA1, (uint8_t *) AudioBuf.u16, AUDIO_BUFFER_SIZE/2 );
      }
      else
      {
        HAL_SAI_Transmit_DMA ( &hsai_BlockA1, (uint8_t *) AudioBuf.u16, AUDIO_BUFFER_SIZE/4 );

        // DMA buffer is mostly wrong after power up, repeat the procedure
        HAL_SAI_DMAStop ( &hsai_BlockA1 );
        HAL_SAI_Transmit_DMA ( &hsai_BlockA1, (uint8_t *) AudioBuf.u16, AUDIO_BUFFER_SIZE/4 );
      }

      HAL_Delay ( 300 );  // TDA 7266 datasheet: 100..200 ms between Standby and Mute

     // HAL_GPIO_WritePin ( GPIOA, AMP_MUTE_Pin, GPIO_PIN_RESET );
    }

    while ( run )
    {
      switch ( bufferOffset )
      {
      case BUFFER_OFFSET_HALF:
        // read next AUDIO_BUFFER_SIZE/2 bytes after 1st part of buffer is played
        run = read_audio_data ( AudioBuf.u8, AUDIO_BUFFER_SIZE/2, &waveFormat );
        bufferOffset = BUFFER_OFFSET_NONE;
        break;

      case BUFFER_OFFSET_FULL:
        // read next AUDIO_BUFFER_SIZE/2 bytes after 2nd part of buffer is played
        run = read_audio_data ( &AudioBuf.u8[AUDIO_BUFFER_SIZE/2], AUDIO_BUFFER_SIZE/2, &waveFormat );
        bufferOffset = BUFFER_OFFSET_NONE;
        break;

      default:
        break;
      }
    }

//    HAL_GPIO_WritePin ( GPIOA, AMP_MUTE_Pin, GPIO_PIN_SET );
    HAL_SAI_DMAStop ( &hsai_BlockA1 );
    f_close ( &fileObj );
  }
}

/**
  @short      Handle the DMA transfer complete interrupt
  @details    This function is called when the requested data\n
              has been completely transferred
  @param[in]  hsai ... SAI1_Block handle
 */

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
	  if ( hsai->Instance == SAI1_Block_A )
	  {
	    bufferOffset = BUFFER_OFFSET_FULL;
	  }
}

/**
  @short      Handle the DMA transfer complete interrupt
  @details    This function is called when half of the\n
              requested buffer has been transferred
  @param[in]  hsai ... SAI1_Block handle
 */

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)  {
	  if ( hsai->Instance == SAI1_Block_A )
	  {
	    bufferOffset = BUFFER_OFFSET_HALF;
	  }

}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
