/*
*         Copyright (c), NXP Semiconductors Bangalore / India
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/

/** \file
* Example Source for NfcrdlibEx6_EMVCo_Loopback.
* This application will configure Reader Library as per Emvco specification and start Emvco polling.
* This loop back application will send SELECT_PPSE command and is used to test Emvco2.3.1a(L1)
* digital compliance.
* Please refer Readme.txt file  for  Hardware Pin Configuration, Software Configuration and steps to build and
* execute the project which is present in the same project directory.
* $Author: Purnank G (ing05193) $
* $Revision: 6114 $ (v4.040.05.011646)
* $Date: 2016-09-22 16:56:53 +0530 (Thu, 22 Sep 2016) $
*
* History:
* PN: Generated 14. May 2015
* BK: Generated 12. Jun 2014
* PC: Generated 25. Nov 2012
*
*/

/**
* Reader Library Headers
*/
#include <phApp_Init.h>

#include "NfcrdlibEx7_EMVCo_Polling.h"

/*******************************************************************************
**   Global Defines
*******************************************************************************/

#define PRINT_RESPONSES		0

static uint8_t TagList[] =
{
		0x5F,
		0x2D,
		0xBF,
		0x0C
};

const uint8_t pdol_tag_list[] =
{
		0x9F,	//Unkown Tag
		0x66,

		0x9F,	//Amount Authorised
		0x02,

		0x9F,	//Amount Other
		0x03,

		0x9F,	//Terminal Country Code
		0x1A,

		0x95,	//Terminal Verification Results
		0xFF,

		0x5F,	//Transaction Currency Code
		0x2A,

		0x9A,	//Transaction Date
		0xFF,

		0x9C,	//Transaction Type
		0xFF,

		0x9F,	//Unpredictable Number
		0x37,

		0x9F,	//Unkown Tag 2
		0x5C
};

enum
{
	PTAG_UNKNOWN_1 = 						0,
	PTAG_AMOUNT_AUTORISED = 				2,
	PTAG_AMOUNT_OTHER = 					4,
	PTAG_TERMINAL_COUNTRY_CODE = 			6,
	PTAG_TERMINAL_VERIFICATION_RESULTS = 	8,
	PTAG_CURRENCY_CODE = 					10,
	PTAG_TRANSACTION_DATE = 				12,
	PTAG_TRANSACTION_TYPE = 				14,
	PTAG_UNPREDICTABLE_NUMBER = 			16,
	PTAG_UNKNOWN_2 =						18
};

enum
{
	POS_STATE_APP_SEL,
	POS_STATE_GPO,
	POS_STATE_READ_REC
};

#define MASTER		0
#define VISA		1

typedef union
{
	struct
	{
		uint8_t cla_u8;
		uint8_t ins_u8;
		uint8_t p1_u8;
		uint8_t p2_u8;
		uint8_t le_u8;
	};
	uint8_t read_record_buf[5];
}Read_Record_Cmd;

typedef struct
{
	uint8_t pdol_found 			: 1;
	uint8_t credit_card_type 	: 1;
	uint8_t reserved			: 6;
	uint8_t record_size_u8;
	uint8_t pdol_len_u8;
	uint8_t command_size_u8;
	uint8_t command[255];
	uint8_t resp[255];
	uint8_t pdol_buf[200];
	uint32_t resp_size_u32;
}NFC_Card_Proccess_T;

typedef struct
{
	uint8_t pan[8];
}Card_Info_T;


Read_Record_Cmd read_record;
NFC_Card_Proccess_T card_data_proccess;
Card_Info_T	card_info;

/*
PDOL -> 9F66049F02069F03069F1A0295055F2A029A039C019F3704

9F66 aaaaaaaa     TTQ
9F02 bbbbbbbbbbbb Amount   000000001000
9F03 cccccccccccc Cashback 000000000000
9F1A dddd         TCC
95   eeeeeeeeee   TVR
5F2A ffff         CC
9A   gggggg       YYMMDD   190325
9C   hh           TT
9F37 iiiiiiii     UN       12121212


9F66 a1 a2 a3 a4

A. a1 - Hex to binary -> 00000000
B. a2 - Hex to binary -> 00000000
C. a3 - Hex to binary -> 00000000
D. a4 - Hex to binary -> 00000000 - RFU (Reserved Future Use)

A. a1 (hex to binary gives 8 numbers)

  8. 0 - Contactless MSD          - Example: 1 - true
  7. 0 - Contactless VSDC         - Example: 1 - true
  6. 0 - Contactless qVSDC        - Example: 1 - true
  5. 0 - EMV contact chip         - Example: 1 - true
  4. 0 - Offline-only reader      - Example: 0 - false
  3. 0 - Online PIN               - Example: 0 - false
  2. 0 - Signature                - Example: 0 - false
  1. 0 - Offline data auth (ODA)  - Example: 0 - false

  Example gives: 11110000. Binary to HEX -> F0

B. a2 (hex to binary gives 8 numbers)

  8. 0 - Require Online Crypt     - Example: 0 - false
  7. 0 - CVM required .           - Example: 0 - false
  6. 0 - Offline PIN support      - Example: 1 - true
  5. 0 - RFU(Reserved Future Use) - Example: 0 - false
  4. 0 - RFU(Reserved Future Use) - Example: 0 - false
  3. 0 - RFU(Reserved Future Use) - Example: 0 - false
  2. 0 - RFU(Reserved Future Use) - Example: 0 - false
  1. 0 - RFU(Reserved Future Use) - Example: 0 - false
Example gives: 00100000. Binary to HEX -> 20

C. a3 (hex to binary gives 8 numbers)

  8. 0 - Issuer update process    - Example: 0 - false
  7. 0 - Mobile functionality     - Example: 1 - true
  6. 0 - RFU(Reserved Future Use) - Example: 0 - false
  5. 0 - RFU(Reserved Future Use) - Example: 0 - false
  4. 0 - RFU(Reserved Future Use) - Example: 0 - false
  3. 0 - RFU(Reserved Future Use) - Example: 0 - false
  2. 0 - RFU(Reserved Future Use) - Example: 0 - false
  1. 0 - RFU(Reserved Future Use) - Example: 0 - false
Example gives: 01000000. Binary to HEX -> 40

D. a4 = RFU(Reserved Future Use) = 00000000 = 00
Gives: 00000000. Binary to HEX -> 00

Putting HEX values together returns: TTQ = F0204000
###################################################
TT
Authorization:   00
Balance inquiry: 31
Sale:            00
Cash:            01
Void:            02
Mobile topup:    57

9F66 aaaaaaaa     TTQ      51004000     TTQ
9F02 bbbbbbbbbbbb Amount   000000001000
9F03 cccccccccccc Cashback 000000000000
9F1A dddd         TCC      0578         ISO 3166 Norway (not same as phone)
95   eeeeeeeeee   TVR
5F2A ffff         CC       0978         ISO 4217 Euro
9A   gggggg       YYMMDD   190325
9C   hh           TT       00           ISO 8583:1987 first 2 digits
9F37 iiiiiiii     UN       12121212
STEP 12 - Terminal verification results (TVR):

https://en.wikipedia.org/wiki/Terminal_verification_results

9F66 e1 e2 e3 e4 e5

A. e1 - Hex to binary -> 00000000
B. e2 - Hex to binary -> 00000000
C. e3 - Hex to binary -> 00000000
D. e4 - Hex to binary -> 00000000
E. e5 - Hex to binary -> 00000000
Each zero can be switched on or off.

A. e1 (hex to binary gives 8 numbers)

  8. 0 - Offline process not performed - Example: 0 - false
  7. 0 - SDA failed                    - Example: 0 - false
  6. 0 - ICC data missing              - Example: 0 - false
  5. 0 - Card number on hotlist        - Example: 0 - false
  4. 0 - DDA failed                    - Example: 0 - false
  3. 0 - CDA failed                    - Example: 0 - false
  2. 0 - RFU (SDA was selected)        - Example: 0 - false
  1. 0 - RFU                           - Example: 0 - false
Example gives: 00000000. Binary to HEX -> 00

B. e2 (hex to binary gives 8 numbers)

  8. 0 - Card/terminal version differ. - Example: 0 - false
  7. 0 - Expired app                   - Example: 0 - false
  6. 0 - App not yet effective         - Example: 0 - false
  5. 0 - Service not allowed for card  - Example: 0 - false
  4. 0 - New card                      - Example: 0 - false
  3. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  2. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  1. 0 - RFU(Reserved Future Use)      - Example: 0 - false
Example gives: 00000000. Binary to HEX -> 00

C. e3 (hex to binary gives 8 numbers)

  8. 0 - Cardholder verification fail  - Example: 0 - false
  7. 0 - Unrecognised CVM              - Example: 0 - false
  6. 0 - PIN try limit exceeded        - Example: 0 - false
  5. 0 - PIN required, but no pinpad   - Example: 0 - false
  4. 0 - PIN req. & present & missing  - Example: 0 - false
  3. 0 - On-line PIN entered           - Example: 0 - false
  2. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  1. 0 - RFU(Reserved Future Use)      - Example: 0 - false
Example gives: 00000000. Binary to HEX -> 00

D. e4 (hex to binary gives 8 numbers)

  8. 0 - Transact. exceeds floor limit - Example: 0 - false
  7. 0 - Lower offline limit exceeded  - Example: 0 - false
  6. 0 - Upper offline limit exceeded  - Example: 0 - false
  5. 0 - Transa. randomly sele. online - Example: 0 - false
  4. 0 - Merch. forced online transac. - Example: 0 - false
  3. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  2. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  1. 0 - RFU(Reserved Future Use)      - Example: 0 - false
Example gives: 00000000. Binary to HEX -> 00

E. e5 (hex to binary gives 8 numbers)

  8. 0 - Default TDOL Used             - Example: 0 - false
  7. 0 - Issuer authentication failed  - Example: 0 - false
  6. 0 - Script fail before final GAC  - Example: 0 - false
  5. 0 - Script fail after final GAC   - Example: 0 - false
  4. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  3. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  2. 0 - RFU(Reserved Future Use)      - Example: 0 - false
  1. 0 - RFU(Reserved Future Use)      - Example: 0 - false
Example gives: 00000000. Binary to HEX -> 00
Putting all HEX values together gives: TVR = 0000000000

STEP 13 - Final Command:
9F66 aaaaaaaa     TTQ      51004000
9F02 bbbbbbbbbbbb Amount   000000001000
9F03 cccccccccccc Cashback 000000000000
9F1A dddd         TCC      0578
95   eeeeeeeeee   TVR      0000000000
5F2A ffff         CC       0978
9A   gggggg       YYMMDD   190325
9C   hh           TT       00
9F37 iiiiiiii     UN       12121212

*/

// 80A8000023832151004000000000001000000000000000079200000000000949220216001212121200
//{0x80, 0xA8, 0x00, 0x00, 0x23, 0x83, 0x21, 0xF0, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x49, 0x22, 0x02, 0x15, 0x00, 0x12, 0x12, 0x12, 0x12, 00};

const uint8_t no_pdol[8] = {0x80, 0xA8, 0x00, 0x00, 0x02, 0x83, 0x00, 0x00};

uint8_t gpo_cmd_size_u8 = 8;

uint8_t emvco_state = POS_STATE_APP_SEL;


/*HAL variables*/
uint8_t                            bHalBufferTx[PHAC_EMVCO_MAX_BUFFSIZE];          /* HAL TX buffer. Size 256 - Based on maximum FSL */
uint8_t                            bHalBufferRx[PHAC_EMVCO_MAX_BUFFSIZE];          /* HAL RX buffer. Size 256 - Based on maximum FSL */
void *pHal;

#ifdef PHOSAL_FREERTOS_STATIC_MEM_ALLOCATION
uint32_t aEmvcoPollingTaskBuffer[EMVCO_POLLING_TASK_STACK];
#else /* PHOSAL_FREERTOS_STATIC_MEM_ALLOCATION */
#define aEmvcoPollingTaskBuffer     NULL
#endif /* PHOSAL_FREERTOS_STATIC_MEM_ALLOCATION */

/*******************************************************************************
**   Static Defines
*******************************************************************************/

#ifdef RUN_TEST_SUIT
/* EMVCo: Select PPSE Command */
static uint8_t PPSE_SELECT_APDU[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59,
        0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };
#else
uint8_t APPSE_respsize;
/*Define Macro for the PPSE command with different P2 value in PPSE command*/
#define PPSE_FCI
//#define PPSE_FMD
//#define PPSE_FCP
//#define PPSE_NO_LE


/* CLA = 0x00
 *  INS = 0xA4
 *  P1 = 0x04
 *  P2 = 0x00
 *  LC = 0x0E,
 *  DF = 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31
 *  LE = 0x00
 */
/*P2 parameter values of
 *  000000xxb = Return FCI template, optional use of FCI tag and length ,
 *  000001xxb = Return FCP template, mandatory use of FCP tag and length,
 *  000010xxb = Return FMD template, mandatory use of FMD tag and length,
 *  000011xxb = No response data if Le field absent, or proprietary if Le field present
 * */
#ifdef PPSE_FCI
/* EMVCo: Select PPSE Command */

static uint8_t APP_SELECT_APDU[13] = {0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10, 0x00};


						//Master Card AID
uint8_t AID_LIST[2][13] = {{0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10, 0x00},
						//VISA Card AID
						 {0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10, 0x00}};




#endif
#ifdef PPSE_FCP
/* EMVCo: Select PPSE Command */
static uint8_t PPSE_SELECT_APDU[] = { 0x00, 0xA4, 0x04, 0x04, 0x0E, 0x32, 0x50, 0x41, 0x59,
        0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };

/*static uint8_t PPSE_SELECT_APDU[] = {0x00, 0xA4, 0x04, 0x04, 0xA0, 0x00, 0x00, 0x00, 0x04,
									 0x10, 0x10, 0x00, 0x90, 0x00};*/
#endif
#ifdef  PPSE_FMD
/* EMVCo: Select PPSE Command */
static uint8_t PPSE_SELECT_APDU[] = { 0x00, 0xA4, 0x04, 0x08, 0x0E, 0x32, 0x50, 0x41, 0x59,
        0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };
#endif
#ifdef PPSE_NO_LE
/* EMVCo: Select PPSE Command */
static uint8_t PPSE_SELECT_APDU[] = { 0x00, 0xA4, 0x04, 0x0C, 0x0E, 0x32, 0x50, 0x41, 0x59,
        0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31 };
#endif
#endif
uint8_t command_buffer[PHAC_EMVCO_MAX_BUFFSIZE];
uint8_t *response_buffer;

/*******************************************************************************
**   Function Declaration
*******************************************************************************/
void Emvco_Polling(void * pHalParams);
static phStatus_t LoadEmvcoSettings();
static phStatus_t EmvcoRfReset(void);
static phStatus_t EmvcoDataLoopBack(phacDiscLoop_Sw_DataParams_t * pDataParams);
static phStatus_t EmvcoDataExchange(uint8_t * com_buffer, uint8_t cmdsize, uint8_t ** resp_buffer, uint32_t * wRxLength);


uint8_t PDOL_Founder(NFC_Card_Proccess_T* card_data);
uint8_t PAN_Parser(NFC_Card_Proccess_T c_data, Card_Info_T* c_inf);
uint8_t AFL_Proccess(NFC_Card_Proccess_T* card_data, Read_Record_Cmd* read_record);
void PDOL_Parser(NFC_Card_Proccess_T* card_data);

/*******************************************************************************
**   Function Definitions
*******************************************************************************/

/*******************************************************************************
**   Main Function
*******************************************************************************/

int main (void)
{
    do
    {
        phStatus_t status = PH_ERR_INTERNAL_ERROR;

        /* Initialize the Controller */
        phPlatform_Controller_Init();

        DEBUG_PRINTF("\n Emvco Example: ");

        phOsal_Init();

        /* Perform Platform Init */
        status = phPlatform_Init(&sPlatform, bHalBufferTx, sizeof(bHalBufferTx), bHalBufferRx, sizeof(bHalBufferRx));
        CHECK_STATUS(status);
        if(status != PH_ERR_SUCCESS) break;

        /* Initialize Reader Library PAL/AL Components */
        status = phApp_RdLibInit();
        CHECK_STATUS(status);

        if(status != PH_ERR_SUCCESS) break;

        /* Set the generic pointer */
        pHal = &sPlatform.sHal;

#ifndef NXPBUILD__PH_OSAL_NULLOS

        phOsal_ThreadObj_t EmvcoPolling;

        EmvcoPolling.pTaskName = (uint8_t *) "EmvcoPolling";
        EmvcoPolling.pStackBuffer = aEmvcoPollingTaskBuffer;
        EmvcoPolling.priority = EMVCO_POLLING_TASK_PRIO;
        EmvcoPolling.stackSizeInNum = EMVCO_POLLING_TASK_STACK;
        phOsal_ThreadCreate(&EmvcoPolling.ThreadHandle, &EmvcoPolling, &Emvco_Polling, &sPlatform.sHal);

        phOsal_StartScheduler();

        DEBUG_PRINTF("RTOS Error : Scheduler exited. \n");

#else
        Emvco_Polling(&sPlatform.sHal);
#endif
    } while(0);

    while(1); //Comes here if initialization failure or scheduler exit due to error

    return 0;
}

void Emvco_Polling(void * pHalParams)
{
    phStatus_t  status;
    uint16_t    wTagsDetected = 0;
    uint8_t     bCidEnabled;
    uint8_t     bCid;
    uint8_t     bNadSupported;
    uint8_t     bFwi;
    uint8_t     bFsdi;
    uint8_t     bFsci;

    phacDiscLoop_Sw_DataParams_t * pDataParams;

    pDataParams = &sDiscLoop;

    /* Load Emvco Default setting */
    status = LoadEmvcoSettings();
    CHECK_STATUS(status);

    /* Perform RF Reset */
    status = EmvcoRfReset();
    CHECK_STATUS(status);

    status = phhalHw_SetConfig(pHal, PHHAL_HW_CONFIG_SET_EMD, PH_OFF);
    CHECK_STATUS(status);

    status = PHAC_DISCLOOP_NO_TECH_DETECTED;

    //Read record init
	read_record.cla_u8 = 0x00;
	read_record.ins_u8 = 0xB2;
	read_record.le_u8 = 0x00;
	//

    while(1)
    {
        do
        {
            if((status & PH_ERR_MASK) != PHAC_DISCLOOP_NO_TECH_DETECTED)
            {
                /* Perform RF Reset */
                status = EmvcoRfReset();
                CHECK_STATUS(status);
            }

            /* Set discovery loop poll state */
            status = phacDiscLoop_SetConfig(pDataParams, PHAC_DISCLOOP_CONFIG_NEXT_POLL_STATE, PHAC_DISCLOOP_POLL_STATE_DETECTION);
            CHECK_STATUS(status);

            /* Start Polling, Function will return once card is activated or any other error has occurred */
            status = phacDiscLoop_Run(pDataParams, PHAC_DISCLOOP_ENTRY_POINT_POLL);

        } while((status & PH_ERR_MASK) != PHAC_DISCLOOP_DEVICE_ACTIVATED); /* Exit on Card detection */

        status = phacDiscLoop_GetConfig(pDataParams, PHAC_DISCLOOP_CONFIG_TECH_DETECTED, &wTagsDetected);
        CHECK_STATUS(status);

        if(PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_A))
        {
            /* Retrieve 14443-4A protocol parameter */
            status = phpalI14443p4a_GetProtocolParams(
                pDataParams->pPal1443p4aDataParams,
                &bCidEnabled,
                &bCid,
                &bNadSupported,
                &bFwi,
                &bFsdi,
                &bFsci);
            CHECK_STATUS(status);

            /* Set 14443-4 protocol parameter */
            status = phpalI14443p4_SetProtocol(
                pDataParams->pPal14443p4DataParams,
                PH_OFF,
                bCid,
                PH_OFF,
                PH_OFF,
                bFwi,
                bFsdi,
                bFsci);
            CHECK_STATUS(status);
        }

        if(PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_B))
        {
            /* Retrieve 14443-3b protocol parameter */
            status = phpalI14443p3b_GetProtocolParams(
                pDataParams->pPal1443p3bDataParams,
                &bCidEnabled,
                &bCid,
                &bNadSupported,
                &bFwi,
                &bFsdi,
                &bFsci);
            CHECK_STATUS(status);

            /* Set 14443-4 protocol parameter */
            status = phpalI14443p4_SetProtocol(
                pDataParams->pPal14443p4DataParams,
                PH_OFF,
                bCid,
                PH_OFF,
                PH_OFF,
                bFwi,
                bFsdi,
                bFsci);
            CHECK_STATUS(status);
        }


        switch(emvco_state)
        {
			case POS_STATE_APP_SEL:
			{
				status = EmvcoDataLoopBack(pDataParams);
			}

			case POS_STATE_GPO:
			{
				if(PDOL_Founder(&card_data_proccess) == FALSE)
				{
					card_data_proccess.command_size_u8 = sizeof(no_pdol);
					memcpy(card_data_proccess.command, no_pdol, card_data_proccess.command_size_u8);
				}

				else
				{
					//TODO: PDOL parser, gpo_cmd
					PDOL_Parser(&card_data_proccess);

				}

				status = EmvcoDataExchange(card_data_proccess.command, card_data_proccess.command_size_u8, &response_buffer, &card_data_proccess.resp_size_u32);
				if (card_data_proccess.resp_size_u32 > 0)
				{
				   memcpy(&card_data_proccess.resp[0], response_buffer, card_data_proccess.resp_size_u32);
#if PRINT_RESPONSES == 1
				   DEBUG_PRINTF("\nState 2 Respone!\n");
				   phApp_Print_Buff(card_data_proccess.resp, (card_data_proccess.resp_size_u32));
#endif

				   if(PAN_Parser(card_data_proccess, &card_info) == TRUE)
				   {
#if PRINT_RESPONSES == 1
					   DEBUG_PRINTF("\n");
					   phApp_Print_Buff(card_info.pan, sizeof(card_info.pan));
#endif

						//
						status = phacDiscLoop_SetConfig(pDataParams, PHAC_DISCLOOP_CONFIG_NEXT_POLL_STATE, PHAC_DISCLOOP_POLL_STATE_REMOVAL);
						CHECK_STATUS(status);
						status = phacDiscLoop_Run(pDataParams, PHAC_DISCLOOP_ENTRY_POINT_POLL);
						//

					   emvco_state = POS_STATE_APP_SEL;
					   break;
				   }

				   else
				   {
					   card_data_proccess.record_size_u8 = AFL_Proccess(&card_data_proccess, &read_record);

					   if(card_data_proccess.record_size_u8 == FALSE)
					   {
						   DEBUG_PRINTF("\nAFL not found or error occured in calculation of record size.\n");
						   emvco_state = POS_STATE_APP_SEL;
						   break;
					   }
				   }
				}

				else
				{
					emvco_state = POS_STATE_APP_SEL;
					break;
				}
			}

			case POS_STATE_READ_REC:
			{
				uint8_t i;

				card_data_proccess.command_size_u8 = 5;
				for(i = 0; i < card_data_proccess.record_size_u8; i++)
				{
					status = EmvcoDataExchange(read_record.read_record_buf, card_data_proccess.command_size_u8, &response_buffer, &card_data_proccess.resp_size_u32);
					if (card_data_proccess.resp_size_u32 > 0)
					{
						memcpy(&card_data_proccess.resp[0], response_buffer, card_data_proccess.resp_size_u32);
#if PRINT_RESPONSES == 1
						DEBUG_PRINTF("\nREAD RECORD RESPONSE\n");
					   phApp_Print_Buff(card_data_proccess.resp, card_data_proccess.resp_size_u32);
#endif

					   if(PAN_Parser(card_data_proccess, &card_info) == TRUE)
					   {
						   DEBUG_PRINTF("\n");
						   phApp_Print_Buff(card_info.pan, sizeof(card_info.pan));
					   }

					   else
					   {
						   DEBUG_PRINTF("\nPAN not found in response.\n");
					   }
					}

					//Record adresi arttırıldı.
					read_record.p1_u8++;
				}

				//
				status = phacDiscLoop_SetConfig(pDataParams, PHAC_DISCLOOP_CONFIG_NEXT_POLL_STATE, PHAC_DISCLOOP_POLL_STATE_REMOVAL);
				CHECK_STATUS(status);
				status = phacDiscLoop_Run(pDataParams, PHAC_DISCLOOP_ENTRY_POINT_POLL);
				//
				emvco_state = POS_STATE_APP_SEL;
				break;
			}
			default: break;
        }
    }
}

/**
* \brief Perform RF Reset as per Emvco Specification
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
* \retval Other Depending on implementation and underlying component.
*/
static phStatus_t EmvcoRfReset(void)
{
    phStatus_t status = PH_ERR_SUCCESS;

    /*RF Field OFF*/
    status = phhalHw_FieldOff(sDiscLoop.pHalDataParams);
    CHECK_STATUS(status);

    status = phhalHw_Wait(sDiscLoop.pHalDataParams,PHHAL_HW_TIME_MICROSECONDS, 5100);
    CHECK_STATUS(status);

    /*RF Field ON*/
    status = phhalHw_FieldOn(sDiscLoop.pHalDataParams);
    CHECK_STATUS(status);

    return status;
}

/**
* \brief Exchange Data APDU Packets for EMVCO (ISO14443-4 Exchange)
* This function will Exchange APDU data packets provided by Loop-Back Application
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
* \retval Other Depending on implementation and underlying component.
*/
static phStatus_t EmvcoDataExchange(uint8_t * com_buffer, uint8_t cmdsize, uint8_t ** resp_buffer, uint32_t * wRxLength)
{
    phStatus_t status;
    uint8_t *ppRxBuffer;
    uint16_t wRxLen = 0;

    status = phpalI14443p4_Exchange(&spalI14443p4, PH_EXCHANGE_DEFAULT, com_buffer, cmdsize, &ppRxBuffer, &wRxLen);
    if (PH_ERR_SUCCESS == status)
    {
        /* set the pointer to the start of the R-APDU */
        *resp_buffer = &ppRxBuffer[0];
    }
    else
    {
        /* Exchange not successful, reset the number of rxd bytes */
        wRxLen = 0x00;
    }

    *wRxLength = wRxLen;

    return status;
}

/**
* \brief EMVCo Loop-Back function
* This Loop-Back function converts each received R-APDU into the next C-APDU (by stripping the
* status words), and sends this C-APDU back to the card simulator.
* Also this function send SELECT_PPSE command after card activation.
* Loop-Back Function exist when EOT (End Of Test) Command is received from the card simulator.
* \return Status code
* \retval #PH_ERR_SUCCESS Operation successful.
* \retval Other Depending on implementation and underlying component.
*/
static phStatus_t EmvcoDataLoopBack(phacDiscLoop_Sw_DataParams_t * pDataParams)
{
    uint32_t cmdsize;
    phStatus_t status;
    uint8_t bEndOfLoopBack = 0;
    uint8_t bRemovalProcedure;
    cmdsize = sizeof(APP_SELECT_APDU);


    //Send APDU
    status = EmvcoDataExchange(APP_SELECT_APDU, cmdsize, &response_buffer, &card_data_proccess.resp_size_u32);
    //

//#ifndef RUN_TEST_SUIT

    /*Check if P1 is 0x04 which means that the data field consists of DF name */
    if(APP_SELECT_APDU[2] == 0x04)
    {
#if PRINT_RESPONSES == 1
        DEBUG_PRINTF("\n DF Name: \n");
        /* DF Size = Total Command size - size of(PDU Header + Expected Len(Le))*/
        phApp_Print_Buff(&APP_SELECT_APDU[5], APP_SELECT_APDU[4]);
#endif
    }

    if (card_data_proccess.resp_size_u32 > 0)
    {
        memcpy(&card_data_proccess.resp[0], response_buffer, card_data_proccess.resp_size_u32);
#if PRINT_RESPONSES == 1
        DEBUG_PRINTF("\n Application Selection Response:\n");
        phApp_Print_Buff(card_data_proccess.resp, (card_data_proccess.resp_size_u32));
#endif

        if(card_data_proccess.resp[card_data_proccess.resp_size_u32 - 2] != 0x90 && card_data_proccess.resp[card_data_proccess.resp_size_u32 - 1] != 0x00)
        {
        	if(card_data_proccess.credit_card_type == MASTER)
        	{
        		card_data_proccess.credit_card_type = VISA;
        		uint8_t i;
        		for(i = 0; i < 13; i++)
        		{
        			APP_SELECT_APDU[i] = AID_LIST[card_data_proccess.credit_card_type][i];
        		}
        	}

        	else
        	{
        		uint8_t i;
        		card_data_proccess.credit_card_type = MASTER;
        		for(i = 0; i < 13; i++)
        		{
        			APP_SELECT_APDU[i] = AID_LIST[card_data_proccess.credit_card_type][i];
        		}
        	}
        	DEBUG_PRINTF("\n Card type changed.:\n");
        }

        else
        {
        	emvco_state = POS_STATE_GPO;
        }
		//Bu kısma "bEndOfLoopBack"i baslatmamak adına return eklendi.
        return status;
        //
    }

 //SDK'da var ancak kullanılmıyor.
 /*   else
    {
        DEBUG_PRINTF("\nFCI not recieved\n");
#ifdef PPSE_NO_LE
        DEBUG_PRINTF("Transaction Done Remove card\n");
#else
        DEBUG_PRINTF("Transaction Failed Replace the card\n");
#endif
    }

#endif

    while (!bEndOfLoopBack)
    {
        if (card_data_proccess.resp_size_u32 > 0)
        {
            if (card_data_proccess.resp_size_u32 >= MiN_VALID_DATA_SIZE)
            {
                // EOT (End Of Test) Command. Exit the loop
                if (eEmdRes_EOT == response_buffer[1])
                {
                    // Second byte = 0x70, stop the loopback
                    bEndOfLoopBack = 1;
                    bRemovalProcedure = PH_ON;
                }
                else if (eEmdRes_SW_0 == response_buffer[card_data_proccess.resp_size_u32 - 2])
                {
                    // Format the card response into a new command without the status word 0x90 0x00
                    cmdsize = card_data_proccess.resp_size_u32 - 2;  // To Remove two bytes of status word
                    memcpy(command_buffer, response_buffer, cmdsize);

                    // Send back(Command) : Received Response - Status_Word
                    status = EmvcoDataExchange(command_buffer, cmdsize, &response_buffer, &card_data_proccess.resp_size_u32);
                }
                else
                {
                    // error Abort Loopback
                    bEndOfLoopBack = 1;
                }
            }
            else//if (respsize <6)
            {
                // re-send the select appli APDU
                status = EmvcoDataExchange(APP_SELECT_APDU, cmdsize, &response_buffer, &card_data_proccess.resp_size_u32);
            	if (card_data_proccess.resp_size_u32 == 0)
                {
                    bEndOfLoopBack = 1;
                }
            }
        }//if(respsize > 0)
        else
        {
            bEndOfLoopBack = 1;
        }
    }//while (!bEndOfLoopBack)

    if(bRemovalProcedure == PH_ON)
    {
        // Set Poll state to perform Tag removal procedure
        status = phacDiscLoop_SetConfig(pDataParams, PHAC_DISCLOOP_CONFIG_NEXT_POLL_STATE, PHAC_DISCLOOP_POLL_STATE_REMOVAL);
        CHECK_STATUS(status);

        status = phacDiscLoop_Run(pDataParams, PHAC_DISCLOOP_ENTRY_POINT_POLL);
        bRemovalProcedure = PH_OFF;
    }*/
    return status;
}

/**
* This function will load/configure Discovery loop and 14443 PAL with default values to support Emvco2.3.1a(L1) digital compliance.
 * Application can read these values from EEPROM area and load/configure Discovery loop via SetConfig
* \param   bProfile      Reader Library Profile
* \note    Values used below are default and is for demonstration purpose.
*/
static phStatus_t LoadEmvcoSettings()
{
    phStatus_t status = PH_ERR_SUCCESS;

    /* Passive Bailout bitmap config. */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_BAIL_OUT, 0x00);
    CHECK_STATUS(status);

    /* Passive CON_DEVICE limit. */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEA_DEVICE_LIMIT, 1);
    CHECK_STATUS(status);

    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_DEVICE_LIMIT, 1);
    CHECK_STATUS(status);

    /* Passive polling Tx Guard times in micro seconds. */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_GTA_VALUE_US, 5100);
    CHECK_STATUS(status);

    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_GTB_VALUE_US, 5100);
    CHECK_STATUS(status);

    /* Configure FSDI for the 14443P4A tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEA_I3P4_FSDI, 0x08);
    CHECK_STATUS(status);

    /* Configure CID for the 14443P4A tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEA_I3P4_CID, 0x00);
    CHECK_STATUS(status);

    /* Configure DRI for the 14443P4A tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEA_I3P4_DRI, 0x00);
    CHECK_STATUS(status);

    /* Configure DSI for the 14443P4A tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEA_I3P4_DSI, 0x00);
    CHECK_STATUS(status);

    /* Configure AFI for the type B tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_AFI_REQ, 0x00);
    CHECK_STATUS(status);

    /* Configure FSDI for the type B tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_FSDI, 0x08);
    CHECK_STATUS(status);

    /* Configure CID for the type B tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_CID, 0x00);
    CHECK_STATUS(status);

    /* Configure DRI for the type B tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_DRI, 0x00);
    CHECK_STATUS(status);

    /* Configure DSI for the type B tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_DSI, 0x00);
    CHECK_STATUS(status);

    /* Configure Extended ATQB support for the type B tags */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_TYPEB_EXTATQB, 0x00);
    CHECK_STATUS(status);

    /* Configure reader library mode */
    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_OPE_MODE, RD_LIB_MODE_EMVCO);
    CHECK_STATUS(status);

    return status;
}

//
uint8_t PDOL_Founder(NFC_Card_Proccess_T* card_data)
{
	card_data->pdol_found = FALSE;
	uint8_t pdol_idx = 0;
	for(uint8_t i = 0; i < card_data->resp_size_u32; i++)
	{

		if(card_data->pdol_found == FALSE)
		{
			//PDOL TAG -> 9F38
			if(card_data->resp[i] == 0x9F && card_data->resp[i + 1] == 0x38)
			{
				//PDOL Found
				card_data->pdol_found = TRUE;
				card_data->pdol_len_u8 = card_data->resp[i + 2];
				i += 3;
			}
		}


		if(card_data->pdol_found == TRUE)
		{
			for(uint8_t j = 0; j < card_data->pdol_len_u8; j++)
			{
				card_data->pdol_buf[pdol_idx] = card_data->resp[i + j];
				pdol_idx++;
			}
			return TRUE;
		}
	}
	return FALSE;
}

void PDOL_Parser(NFC_Card_Proccess_T* card_data)
{
	uint8_t tag_data_len = 0;

	card_data->command[0] = 0x80;
	card_data->command[1] = 0xA8;
	card_data->command[2] = 0x00;
	card_data->command[3] = 0x00;
	//4. index decoded pdol data + 2
	card_data->command[5] = 0x83;
	//6. index decoded pdol data
	uint8_t cmd_idx = 7;
	for(uint8_t i = 0; i < card_data->pdol_len_u8; i++)
	{
		//PDOL TAG -> 9F38
		for(uint8_t j = 0; j < sizeof(pdol_tag_list); j += 2)
		{
			if(card_data->pdol_buf[i] == pdol_tag_list[j] && ((card_data->pdol_buf[i + 1] == pdol_tag_list[j + 1]) || (pdol_tag_list[j + 1] == 0xFF)))
			{
				//Tag Detected
				if(card_data->pdol_buf[i + 1] == pdol_tag_list[j + 1])
				{
					tag_data_len = card_data->pdol_buf[i + 2];
				}

				else if(pdol_tag_list[j + 1] == 0xFF)
				{
					tag_data_len = card_data->pdol_buf[i + 1];
				}

				card_data->command_size_u8 += tag_data_len;

				switch(j)
				{
					case PTAG_UNKNOWN_1:
					{
						//TTQ Calculation
						if(tag_data_len == 4)
						{
							card_data->command[cmd_idx] = 0xF0;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x20;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x40;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
						}
						break;
					}

					case PTAG_AMOUNT_AUTORISED:
					{
						if(tag_data_len == 6)
						{
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x10;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
						}
						break;
					}

					case PTAG_AMOUNT_OTHER:
					{
						if(tag_data_len == 6)
						{
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
						}
						break;
					}

					case PTAG_CURRENCY_CODE:
					{
						if(tag_data_len == 2)
						{
							//Turkish Lira Selected
							card_data->command[cmd_idx] = 0x09;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x49;
							cmd_idx++;
						}
						break;
					}

					case PTAG_TERMINAL_COUNTRY_CODE:
					{
						if(tag_data_len == 2)
						{
							//Turkey Selected
							card_data->command[cmd_idx] = 0x07;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x92;
							cmd_idx++;
						}
						break;
					}

					case PTAG_TERMINAL_VERIFICATION_RESULTS:
					{
						if(tag_data_len == 5)
						{
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
						}
						break;
					}

					case PTAG_TRANSACTION_DATE:
					{
						if(tag_data_len == 3)
						{
							card_data->command[cmd_idx] = 0x22;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x02;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x18;
							cmd_idx++;
						}
						break;
					}

					case PTAG_TRANSACTION_TYPE:
					{
						if(tag_data_len == 1)
						{
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
						}
						break;
					}

					case PTAG_UNPREDICTABLE_NUMBER:
					{
						if(tag_data_len == 4)
						{
							card_data->command[cmd_idx] = 0x12;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x12;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x12;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x12;
							cmd_idx++;
						}
						break;
					}

					case PTAG_UNKNOWN_2:
					{
						if(tag_data_len == 8)
						{
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
							card_data->command[cmd_idx] = 0x00;
							cmd_idx++;
						}
						break;
					}
				}
			}
		}
	}
	card_data->command[4] = cmd_idx - 7 + 2;
	card_data->command[6] = cmd_idx - 7;
	card_data->command[cmd_idx] = 0x00;
	cmd_idx++;
	card_data->command_size_u8 = cmd_idx;
}

uint8_t PAN_Parser(NFC_Card_Proccess_T c_data, Card_Info_T* c_inf)
{
	uint8_t i, j, pan_idx = 0;
	for(i = 0; i < c_data.resp_size_u32; i++)
	{
		//PAN TAG -> 0x5A, eski bir kartta PAN tagi 0x9F, 0x6B olarak görüldü.
		//TRACK 2 EQUIVALENT DATA TAG -> 0x57
		if((c_data.resp[i] == 0x5A || (c_data.resp[i - 1] == 0x9F && c_data.resp[i] == 0x6B)) || c_data.resp[i] == 0x57)
		{
			for(j = 0; j < 8; j++)
			{
				c_inf->pan[pan_idx] = c_data.resp[j + i + 2];
				pan_idx++;
			}
			return TRUE;
		}
	}
	return FALSE;
}

uint8_t AFL_Proccess(NFC_Card_Proccess_T* card_data, Read_Record_Cmd* read_record)
{
	uint8_t i, j, rec_size, afl_found = 0;
	uint8_t afl_buffer[4];
	for(i = 0; i < card_data->resp_size_u32; i++)
	{
		//AFL TAG -> 0x94
		if(card_data->resp[i] == 0x94)
		{
			afl_found = 1;

			for(j = 0; j < 4; j++)
			{
				afl_buffer[j] = card_data->resp[i + j + 2];
			}

			rec_size = afl_buffer[2] - afl_buffer[1] + 1;
			break;
		}
	}

	if(afl_found == 1)
	{
		uint8_t sfi = afl_buffer[0] >> 3;
		read_record->p1_u8 = afl_buffer[1];
		read_record->p2_u8 = (sfi << 3) | 0x04;
		return rec_size;
	}

	else
	{
		return FALSE;
	}
}
//
/* Stubs, in case the phApp_Init.c expects these implementations */
#ifdef NXPBUILD__PHPAL_I14443P4MC_SW
/*
 * WTX Callback called from WTX timer of 14443p3mC PAL.
 */
void pWtoxCallBck(uint8_t bTimerId)
{
  /* Dummy */
}

uint8_t aAppHCEBuf[32];
uint16_t wAppHCEBuffSize = sizeof(aAppHCEBuf);
#endif /* NXPBUILD__PHPAL_I14443P4MC_SW */

#ifdef NXPBUILD__PHPAL_I18092MT_SW
void pRtoxCallBck(uint8_t bTimerId)
{
  /* Dummy */
}
#endif /* NXPBUILD__PHPAL_I18092MT_SW */


#ifdef NXPBUILD__PHHAL_HW_TARGET
/* Stubbed definitions in case TARGET is enabled */
uint8_t  sens_res[2]     = {0x04, 0x00};
uint8_t  nfc_id1[3]      = {0xA1, 0xA2, 0xA3};
uint8_t  sel_res         = 0x40;
uint8_t  nfc_id3         = 0xFA;
uint8_t  poll_res[18]    = {0x01, 0xFE, 0xB2, 0xB3, 0xB4, 0xB5,
                                   0xB6, 0xB7, 0xC0, 0xC1, 0xC2, 0xC3,
                                   0xC4, 0xC5, 0xC6, 0xC7, 0x23, 0x45 };
#endif /* NXPBUILD__PHHAL_HW_TARGET */

/******************************************************************************
**                            End Of File
******************************************************************************/
