/*
 * ClapB7BinaryParser.cpp
 *
 *  Created on: May 28, 2022
 *      Author: yemre
 */

#include <ClapB7BinaryParser.h>
#include "ClapB7Driver.hpp"
#include <cstring>
#include <iostream>

const uint32_t aulCrcTable[256] =
        {
                0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL, 0x076dc419UL, 0x706af48fUL,
                0xe963a535UL, 0x9e6495a3UL, 0x0edb8832UL, 0x79dcb8a4UL, 0xe0d5e91eUL, 0x97d2d988UL,
                0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL, 0x90bf1d91UL, 0x1db71064UL, 0x6ab020f2UL,
                0xf3b97148UL, 0x84be41deUL, 0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL, 0x83d385c7UL,
                0x136c9856UL, 0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL, 0x14015c4fUL, 0x63066cd9UL,
                0xfa0f3d63UL, 0x8d080df5UL, 0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL, 0xa2677172UL,
                0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL, 0x35b5a8faUL, 0x42b2986cUL,
                0xdbbbc9d6UL, 0xacbcf940UL, 0x32d86ce3UL, 0x45df5c75UL, 0xdcd60dcfUL, 0xabd13d59UL,
                0x26d930acUL, 0x51de003aUL, 0xc8d75180UL, 0xbfd06116UL, 0x21b4f4b5UL, 0x56b3c423UL,
                0xcfba9599UL, 0xb8bda50fUL, 0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL, 0xb10be924UL,
                0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL, 0x76dc4190UL, 0x01db7106UL,
                0x98d220bcUL, 0xefd5102aUL, 0x71b18589UL, 0x06b6b51fUL, 0x9fbfe4a5UL, 0xe8b8d433UL,
                0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL, 0xe10e9818UL, 0x7f6a0dbbUL, 0x086d3d2dUL,
                0x91646c97UL, 0xe6635c01UL, 0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL, 0xf262004eUL,
                0x6c0695edUL, 0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL, 0x65b0d9c6UL, 0x12b7e950UL,
                0x8bbeb8eaUL, 0xfcb9887cUL, 0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL, 0xfbd44c65UL,
                0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL, 0x4adfa541UL, 0x3dd895d7UL,
                0xa4d1c46dUL, 0xd3d6f4fbUL, 0x4369e96aUL, 0x346ed9fcUL, 0xad678846UL, 0xda60b8d0UL,
                0x44042d73UL, 0x33031de5UL, 0xaa0a4c5fUL, 0xdd0d7cc9UL, 0x5005713cUL, 0x270241aaUL,
                0xbe0b1010UL, 0xc90c2086UL, 0x5768b525UL, 0x206f85b3UL, 0xb966d409UL, 0xce61e49fUL,
                0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL, 0x59b33d17UL, 0x2eb40d81UL,
                0xb7bd5c3bUL, 0xc0ba6cadUL, 0xedb88320UL, 0x9abfb3b6UL, 0x03b6e20cUL, 0x74b1d29aUL,
                0xead54739UL, 0x9dd277afUL, 0x04db2615UL, 0x73dc1683UL, 0xe3630b12UL, 0x94643b84UL,
                0x0d6d6a3eUL, 0x7a6a5aa8UL, 0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL, 0x7d079eb1UL,
                0xf00f9344UL, 0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL, 0xf762575dUL, 0x806567cbUL,
                0x196c3671UL, 0x6e6b06e7UL, 0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL, 0x67dd4accUL,
                0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL, 0xd6d6a3e8UL, 0xa1d1937eUL,
                0x38d8c2c4UL, 0x4fdff252UL, 0xd1bb67f1UL, 0xa6bc5767UL, 0x3fb506ddUL, 0x48b2364bUL,
                0xd80d2bdaUL, 0xaf0a1b4cUL, 0x36034af6UL, 0x41047a60UL, 0xdf60efc3UL, 0xa867df55UL,
                0x316e8eefUL, 0x4669be79UL, 0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL, 0x5268e236UL,
                0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL, 0xc5ba3bbeUL, 0xb2bd0b28UL,
                0x2bb45a92UL, 0x5cb36a04UL, 0xc2d7ffa7UL, 0xb5d0cf31UL, 0x2cd99e8bUL, 0x5bdeae1dUL,
                0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL, 0x026d930aUL, 0x9c0906a9UL, 0xeb0e363fUL,
                0x72076785UL, 0x05005713UL, 0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL, 0x0cb61b38UL,
                0x92d28e9bUL, 0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL, 0x86d3d2d4UL, 0xf1d4e242UL,
                0x68ddb3f8UL, 0x1fda836eUL, 0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL, 0x18b74777UL,
                0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL, 0x8f659effUL, 0xf862ae69UL,
                0x616bffd3UL, 0x166ccf45UL, 0xa00ae278UL, 0xd70dd2eeUL, 0x4e048354UL, 0x3903b3c2UL,
                0xa7672661UL, 0xd06016f7UL, 0x4969474dUL, 0x3e6e77dbUL, 0xaed16a4aUL, 0xd9d65adcUL,
                0x40df0b66UL, 0x37d83bf0UL, 0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL, 0x30b5ffe9UL,
                0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL, 0xbad03605UL, 0xcdd70693UL,
                0x54de5729UL, 0x23d967bfUL, 0xb3667a2eUL, 0xc4614ab8UL, 0x5d681b02UL, 0x2a6f2b94UL,
                0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL, 0x2d02ef8dUL
        };

// Calculate and return the CRC for usA binary buffer
static uint32_t CalculateCRC32(uint8_t *szBuf, int iSize)
{
    int iIndex;
    uint32_t ulCRC = 0;
    for (iIndex=0; iIndex<iSize; iIndex++)
    {
        ulCRC = aulCrcTable[(ulCRC ^ szBuf[iIndex]) & 0xff] ^ (ulCRC >> 8);
    }
    return ulCRC;
}

void ClapB7Init(ClapB7Controller* p_Controller, const std::function<void()> callBack)
{
    uint16_t i;
    for ( i = 0; i < sizeof(ClapB7Controller); i++)
    {
        ((uint8_t*)p_Controller)[i] = 0;
    }
    p_Controller->Parser = callBack;
}

void ClapB7Parser(ClapB7Controller* p_Controller, const uint8_t* p_Data, uint16_t len)
{
    int i;
    for(i = 0; i < len; i++)
    {
        switch(p_Controller->status)
        {
            case SYNCH1_CONTROL :
            {
                if(FIRST_SYNCH == p_Data[i])
                {
                    p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                    p_Controller->status = SYNCH2_CONTROL;
                }

                break;
            }

            case SYNCH2_CONTROL :
            {
                if(SECOND_SYNCH == p_Data[i])
                {
                    p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                    p_Controller->status = SYNCH3_CONTROL;
                }
                else if(FIRST_SYNCH == p_Data[i])
                {
                    p_Controller->dataIndex = 0;
                    p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                    p_Controller->status = SYNCH2_CONTROL;
                }
                else
                {
                    p_Controller->dataIndex = 0;
                    p_Controller->status = SYNCH1_CONTROL;
                }
                break;
            }

            case SYNCH3_CONTROL :
            {
                if(THIRD_SYNCH == p_Data[i])
                {
                    p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                    p_Controller->status = CLAP_B7_HEADER_LENGTH;
                }
                else if(THIRD_SYNCH_GPGGA == p_Data[i])
                {
                    p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                    p_Controller->status = CLAP_B7_HEADER_ADD_AGRIC;
                }
                else if(FIRST_SYNCH == p_Data[i])
                {
                    p_Controller->dataIndex = 0;
                    p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                    p_Controller->status = SYNCH2_CONTROL;
                }
                else
                {
                    p_Controller->dataIndex = 0;
                    p_Controller->status = SYNCH1_CONTROL;
                }
                break;
            }

            case CLAP_B7_HEADER_LENGTH :
            {
                p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                p_Controller->header.headerLength = p_Data[i];
                p_Controller->status = CLAP_B7_HEADER_ADD;
                break;
            }

            case CLAP_B7_HEADER_ADD :
            {
                p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                if(p_Controller->dataIndex >= p_Controller->header.headerLength)
                {
                    memcpy((uint8_t*)(&(p_Controller->header)), (p_Controller->rawData), sizeof(ClapB7Header));
                    p_Controller->status = CLAP_B7_DATA_ADD;
                }
                break;
            }
                //AGRIC Header
            case CLAP_B7_HEADER_ADD_AGRIC :
            {
                p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                if(p_Controller->dataIndex >= HEADER_LEN_AGRIC)
                {
                    memcpy((uint8_t*)&(p_Controller->header_Agric), (p_Controller->rawData), sizeof(ClapB7Header_Agric));
                    p_Controller->status = CLAP_B7_DATA_ADD_AGRIC;
                }
                break;
            }
                //AGRIC Data
            case CLAP_B7_DATA_ADD_AGRIC :
            {
                p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                if(p_Controller->dataIndex >= p_Controller->header_Agric.msgLen + HEADER_LEN_AGRIC + CRC_LEN)
                {
                    uint32_t crc;
                    memcpy((uint8_t *)&(crc), (uint8_t*)(p_Controller->rawData + p_Controller->header_Agric.msgLen + HEADER_LEN_AGRIC), sizeof(uint32_t));
                    if(CalculateCRC32(p_Controller->rawData, (int32_t)(p_Controller->dataIndex - CRC_LEN)) == crc)
                    {
                        freq_agric++;
                        memcpy(&p_Controller->clap_ArgicData, (p_Controller->rawData + HEADER_LEN_AGRIC), sizeof(ClapB7_AgricMsg_));
                        p_Controller->Parser();
                    }
                    else{
                        //scanned bytes don't contain meaningful data
                    }
                    p_Controller->dataIndex = 0;
                    p_Controller->status = SYNCH1_CONTROL;
                }
                break;
            }

            case CLAP_B7_DATA_ADD :
            {
                p_Controller->rawData[p_Controller->dataIndex++] = p_Data[i];
                if(p_Controller->dataIndex >= p_Controller->header.messageLen + p_Controller->header.headerLength + CRC_LEN)
                {
                    uint32_t crc;
                    memcpy((uint8_t *)&(crc), (uint8_t*)(p_Controller->rawData + p_Controller->header.messageLen + p_Controller->header.headerLength), sizeof(uint32_t));
                    if(CalculateCRC32(p_Controller->rawData, (int32_t)(p_Controller->dataIndex - CRC_LEN)) == crc)
                    {

                        if(p_Controller->header.messageID == RAWIMU_MSG_ID)
                        {
                            freq_rawimu++;
                            memcpy(&p_Controller->clap_RawimuMsgs, (p_Controller->rawData + p_Controller->header.headerLength), sizeof(ClapB7_RawimuMsgs_));
                        }
                        else if(p_Controller->header.messageID == INSPVAX_MSG_ID)
                        {
                            freq_inspvax++;
                            memcpy(&p_Controller->clapData, (p_Controller->rawData + p_Controller->header.headerLength), sizeof(ClapB7_InspvaxMsgs_));
                        }
                        p_Controller->Parser();
                    }
                    else{
                        //scanned bytes don't contain meaningful data
                    }
                    p_Controller->dataIndex = 0;
                    p_Controller->status = SYNCH1_CONTROL;
                }
                break;
            }

            default :
                break;
        }
    }
}
