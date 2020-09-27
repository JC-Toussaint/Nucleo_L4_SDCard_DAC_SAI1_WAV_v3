
/**-----------------------------------------------------------------
    @file       waveplayer.h
    @brief      Interface for playing wave files
    @details    
    @author     Falko Bilz
    @copyright  Copyright(c) 2016-2017 by Falko Bilz
------------------------------------------------------------------*/

#pragma once

#include "ff.h"           // FIL definition
#include <stdint.h>       // uint16_t definition

typedef enum
{
  LittleEndian,
  BigEndian
}Endianness;

typedef struct
{
  uint32_t  RIFFchunksize;
  uint16_t  FormatTag;
  uint16_t  NumChannels;
  uint32_t  SampleRate;
  uint32_t  ByteRate;
  uint16_t  BlockAlign;
  uint16_t  BitsPerSample;
  uint32_t  DataSize;
}
WAVE_FormatTypeDef;

typedef enum
{
  Valid_WAVE_File = 0,
  Unvalid_RIFF_ID,
  Unvalid_WAVE_Format,
  Unvalid_FormatChunk_ID,
  Unsupported_FormatTag,
  Unsupported_Number_Of_Channel,
  Unsupported_Sample_Rate,
  Unsupported_Bits_Per_Sample,
  Unvalid_DataChunk_ID,
  Unsupporetd_ExtraFormatBytes,
  Unvalid_FactChunk_ID
} ErrorCode;

/**
  * @}
  */



/** @defgroup WAVEPLAYER_Exported_Constants
  * @{
  */
#define  CHUNK_ID                            0x52494646  /* correspond to the letters 'RIFF' */
#define  FILE_FORMAT                         0x57415645  /* correspond to the letters 'WAVE' */
#define  FORMAT_ID                           0x666D7420  /* correspond to the letters 'fmt ' */
#define  DATA_ID                             0x64617461  /* correspond to the letters 'data' */
#define  FACT_ID                             0x66616374  /* correspond to the letters 'fact' */
#define  WAVE_FORMAT_PCM                     0x01
#define  FORMAT_CHNUK_SIZE                   0x10
#define  CHANNEL_MONO                        0x01
#define  CHANNEL_STEREO                      0x02
#define  SAMPLE_RATE_8000                    8000
#define  SAMPLE_RATE_16000                   16000
#define  SAMPLE_RATE_11025                   11025
#define  SAMPLE_RATE_22050                   22050
#define  SAMPLE_RATE_32000                   32000
#define  SAMPLE_RATE_44100                   44100
#define  SAMPLE_RATE_48000                   48000
#define  BITS_PER_SAMPLE_8                   8
#define  BITS_PER_SAMPLE_16                  16
#define  WAVE_DUMMY_BYTE                     0xA5

/**-----------------------------------------------------------------
  @brief      struct of RIFF chunk for simplified wave file
------------------------------------------------------------------*/
typedef struct
{
  char                        ChunkID[4];     ///< Byte 0..3
  uint32_t                    FileSize;       ///< Byte 4..7
  char                        riffType[4];    ///< Byte 8..11
  uint32_t                    SubChunk1ID;    ///< Byte 12..15
  uint32_t                    SubChunk1Size;  ///< Byte 16..19
  uint16_t                    formatTag;      ///< Byte 20, 21: Format category
  uint16_t                    channels;       ///< Byte 22, 23: Number of Channels
  uint32_t                    SampleRate;     ///< Byte 24..27: Sampling rate at which each channel should be played
  uint32_t                    AvgByteRate;    ///< Byte 28..31
  uint16_t                    BlockAlign;     ///< Byte 32, 33
  uint16_t                    BitPerSample;   ///< Byte 34, 35
  uint32_t                    SubChunk2ID;    ///< Byte 36..39
  uint32_t                    SubChunk2Size;  ///< Byte 40..43
} T_WAVE_CHUNK;

/**-----------------------------------------------------------------
  @brief      struct of RIFF chunk as union
------------------------------------------------------------------*/
typedef union
{
  T_WAVE_CHUNK                s;
  uint8_t                     u[ sizeof(T_WAVE_CHUNK) ];
} U_WAVE_CHUNK;

void  wave_play_back      ( const char* const fileName );
void  set_volume          ( const uint16_t vol );

extern FIL  fileObj;      // export for sharing because of its huge size of > 2 KB
