#include "app_neo6m.h"
#include "usart.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define NEO6M_RX_LINE_MAX            128U
#define NEO6M_RX_RING_SIZE           512U
#define NEO6M_FIX_TIMEOUT_MS         2000U
#define NEO6M_UBX_MAX_PACKET_SIZE    32U
#define NEO6M_UBX_TX_TIMEOUT_MS      100U

static volatile uint8_t rx_byte;
static char rx_line[NEO6M_RX_LINE_MAX];
static uint16_t rx_index = 0U;

static volatile uint8_t rx_ring[NEO6M_RX_RING_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;
static volatile uint8_t rx_overflow = 0U;

static volatile uint8_t neo6m_initialized = 0U;
static volatile uint8_t neo6m_new_data = 0U;
static volatile uint32_t neo6m_last_sentence_tick = 0U;

static neo6m_data_t neo6m_latest;

static HAL_StatusTypeDef NEO6M_RestartRxIT(void);
static HAL_StatusTypeDef NEO6M_SendUbx(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t length);
static HAL_StatusTypeDef NEO6M_SetMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
static HAL_StatusTypeDef NEO6M_ConfigureReceiver(void);
static uint8_t NEO6M_ChecksumOk(const char *sentence);
static void NEO6M_ProcessLine(char *line);
static void NEO6M_ParseRMC(char *payload);
static void NEO6M_ParseGGA(char *payload);
static void NEO6M_ProcessRxBuffer(void);
static void NEO6M_HandleRxOverflow(void);
static void NEO6M_RxPushByte(uint8_t byte);
static uint8_t NEO6M_RxPopByte(uint8_t *byte);
static uint16_t NEO6M_RingAdvance(uint16_t index);

static int hex_to_int(char c)
{
    if ((c >= '0') && (c <= '9')) return (c - '0');
    if ((c >= 'A') && (c <= 'F')) return (c - 'A' + 10);
    if ((c >= 'a') && (c <= 'f')) return (c - 'a' + 10);
    return -1;
}

static int split_csv(char *str, char *fields[], int max_fields)
{
    int count = 0;
    char *p = str;

    while ((count < max_fields) && (p != NULL))
    {
        fields[count++] = p;
        p = strchr(p, ',');
        if (p != NULL)
        {
            *p = '\0';
            p++;
        }
    }

    return count;
}

static int32_t parse_latlon_to_e7(const char *s, char hemi)
{
    double raw;
    int32_t result;
    int deg;
    double minutes;
    double decimal_deg;

    if ((s == NULL) || (*s == '\0'))
    {
        return 0;
    }

    raw = atof(s);
    deg = (int)(raw / 100.0);
    minutes = raw - ((double)deg * 100.0);
    decimal_deg = (double)deg + (minutes / 60.0);

    if ((hemi == 'S') || (hemi == 'W'))
    {
        decimal_deg = -decimal_deg;
    }

    if (decimal_deg >= 0.0)
    {
        result = (int32_t)(decimal_deg * 10000000.0 + 0.5);
    }
    else
    {
        result = (int32_t)(decimal_deg * 10000000.0 - 0.5);
    }

    return result;
}

static uint16_t parse_u16_scaled_10(const char *s)
{
    double v;

    if ((s == NULL) || (*s == '\0'))
    {
        return 0U;
    }

    v = atof(s) * 10.0;

    if (v < 0.0)
    {
        v = 0.0;
    }
    if (v > 65535.0)
    {
        v = 65535.0;
    }

    return (uint16_t)(v + 0.5);
}

static int16_t parse_i16_scaled_10(const char *s)
{
    double v;

    if ((s == NULL) || (*s == '\0'))
    {
        return 0;
    }

    v = atof(s) * 10.0;

    if (v > 32767.0)
    {
        v = 32767.0;
    }
    if (v < -32768.0)
    {
        v = -32768.0;
    }

    if (v >= 0.0)
    {
        return (int16_t)(v + 0.5);
    }

    return (int16_t)(v - 0.5);
}

static uint16_t knots_to_centi_kmh(const char *s)
{
    double knots;
    double kmh;

    if ((s == NULL) || (*s == '\0'))
    {
        return 0U;
    }

    knots = atof(s);
    kmh = knots * 1.852 * 100.0;

    if (kmh < 0.0)
    {
        kmh = 0.0;
    }
    if (kmh > 65535.0)
    {
        kmh = 65535.0;
    }

    return (uint16_t)(kmh + 0.5);
}

static uint16_t degrees_to_cdeg(const char *s)
{
    double deg;

    if ((s == NULL) || (*s == '\0'))
    {
        return 0U;
    }

    deg = atof(s) * 100.0;

    if (deg < 0.0)
    {
        deg = 0.0;
    }
    if (deg > 65535.0)
    {
        deg = 65535.0;
    }

    return (uint16_t)(deg + 0.5);
}

static uint16_t NEO6M_RingAdvance(uint16_t index)
{
    index++;
    if (index >= NEO6M_RX_RING_SIZE)
    {
        index = 0U;
    }

    return index;
}

static void NEO6M_RxPushByte(uint8_t byte)
{
    uint16_t next_head = NEO6M_RingAdvance(rx_head);

    if (next_head == rx_tail)
    {
        rx_overflow = 1U;
        return;
    }

    rx_ring[rx_head] = byte;
    rx_head = next_head;
}

static uint8_t NEO6M_RxPopByte(uint8_t *byte)
{
    uint8_t has_data = 0U;
    uint32_t primask;

    if (byte == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (rx_head != rx_tail)
    {
        *byte = rx_ring[rx_tail];
        rx_tail = NEO6M_RingAdvance(rx_tail);
        has_data = 1U;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    return has_data;
}

static void NEO6M_HandleRxOverflow(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    rx_tail = rx_head;
    rx_overflow = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    rx_index = 0U;
}

static HAL_StatusTypeDef NEO6M_SendUbx(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t length)
{
    uint8_t packet[NEO6M_UBX_MAX_PACKET_SIZE];
    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    uint16_t i;
    uint16_t packet_len;

    if ((length + 8U) > sizeof(packet))
    {
        return HAL_ERROR;
    }

    packet[0] = 0xB5U;
    packet[1] = 0x62U;
    packet[2] = msg_class;
    packet[3] = msg_id;
    packet[4] = (uint8_t)(length & 0xFFU);
    packet[5] = (uint8_t)(length >> 8);

    for (i = 0U; i < length; i++)
    {
        packet[6U + i] = (payload != NULL) ? payload[i] : 0U;
    }

    for (i = 2U; i < (uint16_t)(6U + length); i++)
    {
        ck_a = (uint8_t)(ck_a + packet[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }

    packet[6U + length] = ck_a;
    packet[7U + length] = ck_b;
    packet_len = (uint16_t)(length + 8U);

    if (HAL_UART_Transmit(&huart2, packet, packet_len, NEO6M_UBX_TX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef NEO6M_SetMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    uint8_t payload[3];

    payload[0] = msg_class;
    payload[1] = msg_id;
    payload[2] = rate;

    return NEO6M_SendUbx(0x06U, 0x01U, payload, sizeof(payload));
}

static HAL_StatusTypeDef NEO6M_ConfigureReceiver(void)
{
    static const uint8_t cfg_rate_5hz[6] = { 0xC8U, 0x00U, 0x01U, 0x00U, 0x01U, 0x00U };
    static const struct
    {
        uint8_t msg_id;
        uint8_t rate;
    } nmea_rate_table[] =
    {
        { 0x00U, 1U }, /* GGA */
        { 0x04U, 1U }, /* RMC */
        { 0x01U, 0U }, /* GLL */
        { 0x02U, 0U }, /* GSA */
        { 0x03U, 0U }, /* GSV */
        { 0x05U, 0U }, /* VTG */
        { 0x06U, 0U }, /* GRS */
        { 0x07U, 0U }, /* GST */
        { 0x08U, 0U }, /* ZDA */
        { 0x09U, 0U }, /* GBS */
        { 0x0AU, 0U }, /* DTM */
        { 0x0EU, 0U }, /* THS */
        { 0x41U, 0U }  /* TXT */
    };
    uint32_t i;

    if (NEO6M_SendUbx(0x06U, 0x08U, cfg_rate_5hz, sizeof(cfg_rate_5hz)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(20U);

    for (i = 0U; i < (sizeof(nmea_rate_table) / sizeof(nmea_rate_table[0])); i++)
    {
        if (NEO6M_SetMessageRate(0xF0U, nmea_rate_table[i].msg_id, nmea_rate_table[i].rate) != HAL_OK)
        {
            return HAL_ERROR;
        }

        HAL_Delay(10U);
    }

    return HAL_OK;
}

static uint8_t NEO6M_ChecksumOk(const char *sentence)
{
    uint8_t checksum = 0U;
    uint8_t rx_checksum;
    const char *p;
    const char *star;
    int hi;
    int lo;

    if ((sentence == NULL) || (sentence[0] != '$'))
    {
        return 0U;
    }

    star = strchr(sentence, '*');
    if ((star == NULL) || ((star - sentence) < 1) || (star[1] == '\0') || (star[2] == '\0'))
    {
        return 0U;
    }

    p = sentence + 1;
    while (p < star)
    {
        checksum ^= (uint8_t)(*p);
        p++;
    }

    hi = hex_to_int(star[1]);
    lo = hex_to_int(star[2]);

    if ((hi < 0) || (lo < 0))
    {
        return 0U;
    }

    rx_checksum = (uint8_t)((hi << 4) | lo);

    return (checksum == rx_checksum) ? 1U : 0U;
}

static void NEO6M_ParseRMC(char *payload)
{
    char *fields[16] = {0};
    int n;

    n = split_csv(payload, fields, 16);
    if (n < 10)
    {
        return;
    }

    if (fields[2][0] == 'A')
    {
        neo6m_latest.fix_valid = 1U;
    }
    else
    {
        neo6m_latest.fix_valid = 0U;
    }

    if ((fields[3][0] != '\0') && (fields[4][0] != '\0'))
    {
        neo6m_latest.latitude_e7 = parse_latlon_to_e7(fields[3], fields[4][0]);
    }

    if ((fields[5][0] != '\0') && (fields[6][0] != '\0'))
    {
        neo6m_latest.longitude_e7 = parse_latlon_to_e7(fields[5], fields[6][0]);
    }

    neo6m_latest.speed_centi_kmh = knots_to_centi_kmh(fields[7]);
    neo6m_latest.course_cdeg = degrees_to_cdeg(fields[8]);

    if (strlen(fields[1]) >= 6U)
    {
        neo6m_latest.hour   = (uint8_t)(((fields[1][0] - '0') * 10) + (fields[1][1] - '0'));
        neo6m_latest.minute = (uint8_t)(((fields[1][2] - '0') * 10) + (fields[1][3] - '0'));
        neo6m_latest.second = (uint8_t)(((fields[1][4] - '0') * 10) + (fields[1][5] - '0'));
    }

    if (strlen(fields[9]) >= 6U)
    {
        neo6m_latest.day   = (uint8_t)(((fields[9][0] - '0') * 10) + (fields[9][1] - '0'));
        neo6m_latest.month = (uint8_t)(((fields[9][2] - '0') * 10) + (fields[9][3] - '0'));
        neo6m_latest.year  = (uint16_t)(2000U + ((fields[9][4] - '0') * 10U) + (fields[9][5] - '0'));
    }

    neo6m_latest.valid = 1U;
    neo6m_new_data = 1U;
    neo6m_last_sentence_tick = HAL_GetTick();
}

static void NEO6M_ParseGGA(char *payload)
{
    char *fields[16] = {0};
    int n;

    n = split_csv(payload, fields, 16);
    if (n < 10)
    {
        return;
    }

    neo6m_latest.fix_quality = (uint8_t)atoi(fields[6]);
    neo6m_latest.satellites  = (uint8_t)atoi(fields[7]);
    neo6m_latest.hdop_x10    = parse_u16_scaled_10(fields[8]);
    neo6m_latest.altitude_dm = parse_i16_scaled_10(fields[9]);

    if (neo6m_latest.fix_quality == 0U)
    {
        neo6m_latest.fix_valid = 0U;
    }

    neo6m_latest.valid = 1U;
    neo6m_new_data = 1U;
    neo6m_last_sentence_tick = HAL_GetTick();
}

static void NEO6M_ProcessLine(char *line)
{
    char work[NEO6M_RX_LINE_MAX];
    char *star;

    if ((line == NULL) || (line[0] != '$'))
    {
        return;
    }

    if (NEO6M_ChecksumOk(line) == 0U)
    {
        return;
    }

    memset(work, 0, sizeof(work));
    strncpy(work, line + 1, sizeof(work) - 1U);

    star = strchr(work, '*');
    if (star == NULL)
    {
        return;
    }
    *star = '\0';

    if ((strncmp(work, "GPRMC", 5) == 0) || (strncmp(work, "GNRMC", 5) == 0))
    {
        NEO6M_ParseRMC(work);
    }
    else if ((strncmp(work, "GPGGA", 5) == 0) || (strncmp(work, "GNGGA", 5) == 0))
    {
        NEO6M_ParseGGA(work);
    }
    else
    {
        /* ignore other sentences */
    }
}

static void NEO6M_ProcessRxBuffer(void)
{
    uint8_t byte;
    char c;

    if (rx_overflow != 0U)
    {
        NEO6M_HandleRxOverflow();
    }

    while (NEO6M_RxPopByte(&byte) != 0U)
    {
        c = (char)byte;

        if (c == '$')
        {
            rx_index = 0U;
            rx_line[rx_index++] = c;
        }
        else if (c == '\n')
        {
            if ((rx_index > 0U) && (rx_index < NEO6M_RX_LINE_MAX))
            {
                rx_line[rx_index] = '\0';
                NEO6M_ProcessLine(rx_line);
            }

            rx_index = 0U;
        }
        else if (c == '\r')
        {
            /* ignore carriage return */
        }
        else if (rx_index != 0U)
        {
            if (rx_index < (NEO6M_RX_LINE_MAX - 1U))
            {
                rx_line[rx_index] = c;
                rx_index++;
            }
            else
            {
                rx_index = 0U;
            }
        }
        else
        {
            /* wait for start of sentence */
        }
    }
}

static HAL_StatusTypeDef NEO6M_RestartRxIT(void)
{
    return HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1U);
}

HAL_StatusTypeDef NEO6M_App_Init(void)
{
    memset((void *)&neo6m_latest, 0, sizeof(neo6m_latest));
    memset((void *)rx_line, 0, sizeof(rx_line));
    memset((void *)rx_ring, 0, sizeof(rx_ring));

    rx_index = 0U;
    rx_head = 0U;
    rx_tail = 0U;
    rx_overflow = 0U;
    neo6m_new_data = 0U;
    neo6m_last_sentence_tick = 0U;
    neo6m_initialized = 0U;

    if (NEO6M_RestartRxIT() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (NEO6M_ConfigureReceiver() != HAL_OK)
    {
        return HAL_ERROR;
    }

    neo6m_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef NEO6M_App_ReadLatest(neo6m_data_t *data)
{
    uint32_t primask;

    if ((data == NULL) || (neo6m_initialized == 0U))
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(data, (const void *)&neo6m_latest, sizeof(neo6m_data_t));
    if (primask == 0U)
    {
        __enable_irq();
    }

    return HAL_OK;
}

uint8_t NEO6M_App_HasFix(void)
{
    return neo6m_latest.fix_valid;
}

uint8_t NEO6M_App_HasNewData(void)
{
    return neo6m_new_data;
}

void NEO6M_App_ClearNewData(void)
{
    neo6m_new_data = 0U;
}

void NEO6M_App_Process(void)
{
    if (neo6m_initialized == 0U)
    {
        return;
    }

    NEO6M_ProcessRxBuffer();

    if ((neo6m_last_sentence_tick != 0U) &&
        ((HAL_GetTick() - neo6m_last_sentence_tick) > NEO6M_FIX_TIMEOUT_MS))
    {
        neo6m_latest.fix_valid = 0U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    NEO6M_RxPushByte(rx_byte);

    if (NEO6M_RestartRxIT() != HAL_OK)
    {
        neo6m_initialized = 0U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    rx_index = 0U;
    rx_head = 0U;
    rx_tail = 0U;
    rx_overflow = 0U;

    if (NEO6M_RestartRxIT() != HAL_OK)
    {
        neo6m_initialized = 0U;
    }
}
