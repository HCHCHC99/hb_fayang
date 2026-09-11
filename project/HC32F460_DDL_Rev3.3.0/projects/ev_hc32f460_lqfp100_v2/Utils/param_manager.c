/**
 *******************************************************************************
 * @file  param_manager.c
 * @brief Flash parameter storage engine with wear-leveling.
 *        Supports sequential append + CRC32 verification + rollback search.
 *
 *        Multi-instance: each caller provides its own Param_Config_t (sector
 *        range, struct layout, magic) and Param_Runtime_t (current position).
 *        No static state — the same code manages independent Flash regions.
 *******************************************************************************
 */

#include "param_manager.h"
#include <string.h>

#define SECTOR_SIZE             0x2000
#define MAX_ERASE_LIFE          10000
#define MAX_WRITE_RETRY         7

#define PARAM_ENTER_CRITICAL()  __disable_irq()
#define PARAM_EXIT_CRITICAL()   __enable_irq()

Param_Runtime_t g_ParamRuntime;

/*=============================================================================
 * Internal helpers
 *============================================================================*/

/**
 * @brief  Read a uint32_t field from buffer at given byte offset.
 */
static uint32_t GetField32(const void *pBuf, uint32_t offset)
{
    return *(const uint32_t *)((const uint8_t *)pBuf + offset);
}

/**
 * @brief  Write a uint32_t field to buffer at given byte offset.
 */
static void SetField32(void *pBuf, uint32_t offset, uint32_t value)
{
    *(uint32_t *)((uint8_t *)pBuf + offset) = value;
}

/**
 * @brief  Calculate CRC32 for parameter struct (excluding checksum field and tail).
 * @note   Uses DDL CRC-32 hardware module. Calculates word_len = checksumOffset/4 words.
 */
static uint32_t CalcParamCRC(const void *pBuf, uint32_t paramSize, uint32_t checksumOffset)
{
    uint32_t crc_result = 0;
    uint32_t word_len;
    stc_crc_init_t stcCrcInit;

    if (pBuf == NULL) {
        return 0;
    }

    PARAM_ENTER_CRITICAL();

    /* Calculate CRC for all words before checksum field */
    word_len = checksumOffset / 4;

    /* Configure CRC-32 */
    stcCrcInit.u32Protocol = CRC_CRC32;
    stcCrcInit.u32InitValue = 0xFFFFFFFFU;
    stcCrcInit.u32RefIn  = CRC_REFIN_ENABLE;
    stcCrcInit.u32RefOut = CRC_REFOUT_ENABLE;
    stcCrcInit.u32XorOut = CRC_XOROUT_ENABLE;

    CRC_Init(&stcCrcInit);
    CRC_CRC32_AccumulateData(CRC_DATA_WIDTH_32BIT, (const uint32_t *)pBuf, word_len, &crc_result);

    PARAM_EXIT_CRITICAL();

    PARAM_DBG("CalcParamCRC: word_len=%lu, result=0x%08lX\r\n", word_len, crc_result);

    return crc_result;
}

/**
 * @brief  Update runtime debug info (called after init/save).
 */
static void UpdateRuntime(Param_Runtime_t *pRuntime, int32_t res, uint32_t paramSize)
{
    if (pRuntime == NULL) return;

    // PARAM_ENTER_CRITICAL();
    pRuntime->last_res = res;
    if (res == PARAM_OK) {
        pRuntime->save_count++;
    }
    // PARAM_EXIT_CRITICAL();
}

/*=============================================================================
 * Low-level Flash operations (thin wrappers around DDL)
 *============================================================================*/

static int32_t Internal_Erase(uint32_t address)
{
    HC32FLASH_STATUS status;

    PARAM_ENTER_CRITICAL();
    status = HC32FLASH_EraseSector(address);
    PARAM_EXIT_CRITICAL();

    return (status == HC32FLASH_OK) ? PARAM_OK : PARAM_ERR;
}

static uint32_t Internal_ReadWord(uint32_t addr)
{
    uint32_t data;
    PARAM_ENTER_CRITICAL();
    data = HC32FLASH_ReaddWord(addr);
    PARAM_EXIT_CRITICAL();
    return data;
}

static int32_t Internal_WriteWord(uint32_t addr, uint32_t data)
{
    HC32FLASH_STATUS status;

    PARAM_ENTER_CRITICAL();
    status = HC32FLASH_WritedWord_Check(addr, data);
    PARAM_EXIT_CRITICAL();

    return (status == HC32FLASH_OK) ? PARAM_OK : PARAM_ERR;
}

static int32_t Internal_WriteBuffer(uint32_t addr, const uint32_t *buffer, uint32_t word_count)
{
    uint32_t i;
    for (i = 0; i < word_count; i++) {
        if (Internal_WriteWord(addr + i * 4, buffer[i]) != PARAM_OK) {
            return PARAM_ERR;
        }
    }
    return PARAM_OK;
}

static bool Internal_VerifyBuffer(uint32_t addr, const uint32_t *buffer, uint32_t word_count)
{
    uint32_t i;
    for (i = 0; i < word_count; i++) {
        if (Internal_ReadWord(addr + i * 4) != buffer[i]) {
            return false;
        }
    }
    return true;
}

/*=============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief  Initialize parameter storage.
 *         Scans Flash sectors from secStart down to secEnd,
 *         finds the newest valid record (by sequence_id + head/tail magic + CRC),
 *         and loads it into RAM.
 */
int32_t Param_Init(const Param_Config_t *pConfig,
                   Param_Runtime_t *pRuntime,
                   void (*pSetDefaults)(void))
{
    uint32_t max_seq  = 0;
    uint32_t best_addr = 0;
    uint16_t best_sec  = pConfig->secStart;
    bool found = false;
    int  s;
    uint32_t paramWords;
    uint32_t headMagic;
    uint32_t tailMagic;
    uint32_t seq;
    uint32_t calcCrc;
    uint32_t storedCrc;

    if ((pConfig == NULL) || (pConfig->pParamBuf == NULL) || (pSetDefaults == NULL) || (pRuntime == NULL)) {
        return PARAM_ERR_INVD_PARAM;
    }

    paramWords = pConfig->paramSize / 4;

    PARAM_DBG("Param init start, size=%lu, sec=%d..%d\r\n",
              pConfig->paramSize, pConfig->secStart, pConfig->secEnd);

    for (s = pConfig->secStart; s >= pConfig->secEnd; s--) {
        uint32_t addr = s * SECTOR_SIZE;
        uint32_t sector_end = (s + 1) * SECTOR_SIZE;

        while (addr + pConfig->paramSize <= sector_end) {
            headMagic = Internal_ReadWord(addr);

            if (headMagic == pConfig->magicHead) {
                tailMagic = Internal_ReadWord(addr + pConfig->paramSize - 4);
                seq = Internal_ReadWord(addr + pConfig->seqOffset);

                PARAM_DBG("  Found block at 0x%08lX: head=0x%08lX, seq=%lu, tail=0x%08lX\r\n",
                          addr, headMagic, seq, tailMagic);

                if (tailMagic == pConfig->magicTail) {
                    /* Read entire block to temp buffer for CRC check */
                    uint8_t tempBuf[256];
                    uint32_t i;
                    uint32_t *pDest = (uint32_t *)tempBuf;

                    for (i = 0; i < paramWords; i++) {
                        pDest[i] = Internal_ReadWord(addr + i * 4);
                    }

                    /* Calculate and verify CRC */
                    calcCrc = CalcParamCRC(tempBuf, pConfig->paramSize, pConfig->checksumOffset);
                    storedCrc = GetField32(tempBuf, pConfig->checksumOffset);

                    PARAM_DBG("    CRC: stored=0x%08lX, calculated=0x%08lX\r\n",
                              storedCrc, calcCrc);

                    if (calcCrc == storedCrc) {
                        PARAM_DBG("    -> VALID block (CRC passed)\r\n");
                        if (!found || seq > max_seq) {
                            max_seq   = seq;
                            best_addr = addr;
                            best_sec  = (uint16_t)s;
                            found     = true;
                            PARAM_DBG("    -> New best: seq=%lu, addr=0x%08lX\r\n",
                                      max_seq, best_addr);
                        }
                    } else {
                        PARAM_DBG("    -> INVALID block (CRC mismatch), skipping\r\n");
                    }
                } else {
                    PARAM_DBG("    -> INVALID block (tail mismatch: 0x%08lX)\r\n", tailMagic);
                }
                addr += pConfig->paramSize;
            } else if (headMagic == 0xFFFFFFFF) {
                PARAM_DBG("  Hit empty area at 0x%08lX, stop scanning sector\r\n", addr);
                break;
            } else {
                addr += 4;
            }
        }
    }

    if (found) {
        /* Load best record into RAM buffer */
        uint32_t *pDest = (uint32_t *)pConfig->pParamBuf;
        uint32_t i;
        for (i = 0; i < paramWords; i++) {
            pDest[i] = Internal_ReadWord(best_addr + i * 4);
        }

        pRuntime->curr_sec  = best_sec;
        pRuntime->curr_addr = best_addr;

        PARAM_DBG("Param load SUCCESS: seq=%lu, addr=0x%08lX, sec=%lu\r\n",
                  max_seq, best_addr, best_sec);
    } else {
        PARAM_DBG("No valid param block found, using defaults and writing to Flash\r\n");
        pSetDefaults();
        Internal_Erase(pConfig->secStart * SECTOR_SIZE);
        pRuntime->curr_sec  = pConfig->secStart;
        pRuntime->curr_addr = pConfig->secStart * SECTOR_SIZE;
        SetField32(pConfig->pParamBuf, pConfig->seqOffset, 0);
        SetField32(pConfig->pParamBuf, pConfig->eraseCntOffset, 1);

        /* Save defaults to Flash */
        Param_Save(pConfig, pRuntime);

        PARAM_DBG("Defaults saved to Flash at addr=0x%08lX\r\n", pRuntime->curr_addr);
    }

    UpdateRuntime(pRuntime, PARAM_OK, pConfig->paramSize);
    return PARAM_OK;
}

/**
 * @brief  Save parameters to Flash (sequential append with wear-leveling).
 */
int32_t Param_Save(const Param_Config_t *pConfig, Param_Runtime_t *pRuntime)
{
    uint8_t  retry = 0;
    uint32_t write_addr;
    uint32_t paramWords;
    uint32_t seq;

    if ((pConfig == NULL) || (pConfig->pParamBuf == NULL) || (pRuntime == NULL)) {
        return PARAM_ERR_INVD_PARAM;
    }

    paramWords = pConfig->paramSize / 4;
    seq = GetField32(pConfig->pParamBuf, pConfig->seqOffset);

    PARAM_DBG("Param save start, old_seq=%lu\r\n", seq);

    while (retry < MAX_WRITE_RETRY) {
        write_addr = pRuntime->curr_addr;

        /* Check if write would overflow current sector */
        if (write_addr + pConfig->paramSize > (pRuntime->curr_sec + 1) * SECTOR_SIZE) {
            uint32_t eraseCnt = GetField32(pConfig->pParamBuf, pConfig->eraseCntOffset);

            if (eraseCnt < MAX_ERASE_LIFE) {
                if (Internal_Erase(pRuntime->curr_sec * SECTOR_SIZE) == PARAM_OK) {
                    write_addr = pRuntime->curr_sec * SECTOR_SIZE;
                    SetField32(pConfig->pParamBuf, pConfig->eraseCntOffset, eraseCnt + 1);
                } else {
                    retry++;
                    continue;
                }
            } else {
                /* Current sector worn out, move to next sector */
                pRuntime->curr_sec = (pRuntime->curr_sec <= pConfig->secEnd) ?
                                     pConfig->secStart : (pRuntime->curr_sec - 1);
                write_addr = pRuntime->curr_sec * SECTOR_SIZE;
                Internal_Erase(pRuntime->curr_sec * SECTOR_SIZE);
                SetField32(pConfig->pParamBuf, pConfig->eraseCntOffset, 1);
            }
        }

        /* Update metadata in buffer */
        seq = GetField32(pConfig->pParamBuf, pConfig->seqOffset);
        SetField32(pConfig->pParamBuf, pConfig->seqOffset, seq + 1);
        SetField32(pConfig->pParamBuf, pConfig->checksumOffset,
                   CalcParamCRC(pConfig->pParamBuf, pConfig->paramSize, pConfig->checksumOffset));

        if (Internal_WriteBuffer(write_addr, (const uint32_t *)pConfig->pParamBuf, paramWords) == PARAM_OK) {
            if (Internal_VerifyBuffer(write_addr, (const uint32_t *)pConfig->pParamBuf, paramWords)) {
                /* Write successful — advance position */
                pRuntime->curr_addr = write_addr + pConfig->paramSize;

                PARAM_DBG("Param save SUCCESS, new_seq=%lu, addr=0x%08lX\r\n",
                          GetField32(pConfig->pParamBuf, pConfig->seqOffset), write_addr);

                UpdateRuntime(pRuntime, PARAM_OK, pConfig->paramSize);
                return PARAM_OK;
            }
        }

        retry++;
        /* On failure, advance write pointer to skip the bad spot */
        pRuntime->curr_addr = write_addr + pConfig->paramSize;
    }

    UpdateRuntime(pRuntime, PARAM_ERR, pConfig->paramSize);
    PARAM_DBG("Param save FAILED after %d retries\r\n", MAX_WRITE_RETRY);
    return PARAM_ERR;
}

/**
 * @brief  Debug: erase all parameter sectors and re-initialize with defaults.
 */
void Param_Debug_EraseAll(const Param_Config_t *pConfig,
                          Param_Runtime_t *pRuntime,
                          void (*pSetDefaults)(void))
{
    int s;
    PARAM_DBG("Erase all param sectors start\r\n");

    for (s = pConfig->secStart; s >= pConfig->secEnd; s--) {
        Internal_Erase(s * SECTOR_SIZE);
    }

    Param_Init(pConfig, pRuntime, pSetDefaults);
    PARAM_DBG("Erase all param sectors done\r\n");
}

/**
 * @brief  Public wrapper: erase a single sector.
 */
int32_t Param_EraseSector(uint32_t address)
{
    return Internal_Erase(address);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
