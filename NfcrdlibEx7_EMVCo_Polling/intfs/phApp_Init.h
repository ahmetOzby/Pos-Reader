/*
*         Copyright (c), NXP Semiconductors Gratkorn / Austria
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
* Cards identification.
* $Author: Kiran Kumar Makam Nataraja (nxp69453) $
* $Revision: 4423 $ (v4.040.05.011646)
* $Date: 2016-04-22 11:50:54 +0530 (Fri, 22 Apr 2016) $
*
* History:
*
*
*/

#ifndef PHAPP_INIT_H
#define PHAPP_INIT_H

/* Status header */
#include <ph_Status.h>

/* Platform Header */
#include <phPlatform.h>

/* PAL Headers */
#include <phpalI14443p3a.h>
#include <phpalI14443p3b.h>
#include <phpalI14443p4a.h>
#include <phpalI14443p4.h>
#include <phpalMifare.h>
#include <phpalFelica.h>
#include <phpalSli15693.h>
#include <phpalI18000p3m3.h>
#include <phpalEpcUid.h>
#include <phpalI18092mPI.h>
#include <phpalI14443p4mC.h>
#include <phpalI18092mT.h>

/* Discovery Loop Header */
#include <phacDiscLoop.h>

/* AL Headers */
#include <phalFelica.h>
#include <phalMfc.h>
#include <phalMful.h>
#include <phalMfdf.h>
#include <phalT1T.h>
#include <phalTop.h>
#include <phalI15693.h>
#include <phalSli.h>
#include <phalI18000p3m3.h>

/* HCE header */
#include <phceT4T.h>

/* LLCP header */
#include <phlnLlcp.h>

/* SNEP header */
#include <phnpSnep.h>

/* Crypto Headers */
#ifdef NXPBUILD__PH_CRYPTOSYM_SW
#include <phCryptoSym.h>
#endif /* NXPBUILD__PH_CRYPTOSYM_SW */

#ifdef NXPBUILD__PH_CRYPTORNG_SW
#include <phCryptoRng.h>
#endif /* NXPBUILD__PH_CRYPTORNG_SW */

#ifdef NXPBUILD__PH_KINETIS
#   include <fsl_debug_console.h>
#endif

#ifdef   DEBUG
    #ifdef NXPBUILD__PH_KINETIS_K82
        #define  DEBUG_PRINTF   DbgConsole_Printf
    #else /* NXPBUILD__PH_KINETIS_K82 */
        #include <stdio.h>
        #define  DEBUG_PRINTF(...) printf(__VA_ARGS__); fflush(stdout)
    #endif /* NXPBUILD__PH_KINETIS_K82 */
#else /* DEBUG */
    #define  DEBUG_PRINTF(...)
#endif /* DEBUG */

/*******************************************************************************
**   Global Variable Declaration
*******************************************************************************/
extern phPlatform_DataParams_t            sPlatform;          /* Platform component holder */

/* PAL declarations */
#ifdef NXPBUILD__PHPAL_I14443P3A_SW
extern phpalI14443p3a_Sw_DataParams_t     spalI14443p3a;      /* PAL ISO I14443-A component */
#endif /* NXPBUILD__PHPAL_I14443P3A_SW */

#ifdef NXPBUILD__PHPAL_I14443P3B_SW
extern phpalI14443p3b_Sw_DataParams_t     spalI14443p3b;      /* PAL ISO I14443-B component */
#endif /* NXPBUILD__PHPAL_I14443P3B_SW */

#ifdef NXPBUILD__PHPAL_I14443P4A_SW
extern phpalI14443p4a_Sw_DataParams_t     spalI14443p4a;      /* PAL ISO I14443-4A component */
#endif /* NXPBUILD__PHPAL_I14443P4A_SW */

#ifdef NXPBUILD__PHPAL_I14443P4_SW
extern phpalI14443p4_Sw_DataParams_t      spalI14443p4;       /* PAL ISO I14443-4 component */
#endif /* NXPBUILD__PHPAL_I14443P4_SW */

#ifdef NXPBUILD__PHPAL_MIFARE_SW
extern phpalMifare_Sw_DataParams_t        spalMifare;         /* PAL Mifare component */
#endif /* NXPBUILD__PHPAL_MIFARE_SW */

#ifdef NXPBUILD__PHPAL_FELICA_SW
extern phpalFelica_Sw_DataParams_t        spalFelica;         /* PAL Felica component */
#endif /* NXPBUILD__PHPAL_FELICA_SW */

#ifdef NXPBUILD__PHPAL_SLI15693_SW
extern phpalSli15693_Sw_DataParams_t      spalSli15693;       /* PAL ISO 15693 component */
#endif /* NXPBUILD__PHPAL_SLI15693_SW */

#ifdef NXPBUILD__PHPAL_I18000P3M3_SW
extern phpalI18000p3m3_Sw_DataParams_t    spalI18000p3m3;     /* PAL ISO 18000p3m3 component */
#endif /* NXPBUILD__PHPAL_I18000P3M3_SW */

#ifdef NXPBUILD__PHPAL_EPCUID_SW
extern phpalEpcUid_Sw_DataParams_t        spalEpcUid;         /* PAL EPC UID component */
#endif /* NXPBUILD__PHPAL_EPCUID_SW */

#ifdef NXPBUILD__PHPAL_I18092MPI_SW
extern phpalI18092mPI_Sw_DataParams_t     spalI18092mPI;      /* PAL ISO 18092 Initiator component */
#endif /* NXPBUILD__PHPAL_I18092MPI_SW */

#ifdef NXPBUILD__PHPAL_I14443P4MC_SW
extern phpalI14443p4mC_Sw_DataParams_t    spalI14443p4mC;     /* PAL ISO 14443 MC Target component */
#endif /* NXPBUILD__PHPAL_I14443P4MC_SW */

#ifdef NXPBUILD__PHPAL_I18092MT_SW
extern phpalI18092mT_Sw_DataParams_t      spalI18092mT;       /* PAL ISO 18092 Target component */
#endif /* NXPBUILD__PHPAL_I18092MT_SW */

/* Discovery Loop variable */
#ifdef NXPBUILD__PHAC_DISCLOOP_SW
extern phacDiscLoop_Sw_DataParams_t       sDiscLoop;          /* Discovery loop component */
#endif /* NXPBUILD__PHAC_DISCLOOP_SW */

/* Application Layer declaration */
#ifdef NXPBUILD__PHAL_FELICA_SW
extern phalFelica_Sw_DataParams_t         salFelica;          /* AL Felica component */
#endif /* NXPBUILD__PHAL_FELICA_SW */

#ifdef NXPBUILD__PHAL_MFC_SW
extern phalMfc_Sw_DataParams_t            salMFC;             /* AL Mifare Classic component */
#endif /* NXPBUILD__PHAL_MFC_SW */

#ifdef NXPBUILD__PHAL_MFUL_SW
extern phalMful_Sw_DataParams_t           salMFUL;            /* AL Mifare Ultralight component */
#endif /* NXPBUILD__PHAL_MFUL_SW */

#ifdef NXPBUILD__PHAL_MFDF_SW
extern phalMfdf_Sw_DataParams_t           salMFDF;            /* AL Mifare DESFire component */
#endif /* NXPBUILD__PHAL_MFDF_SW */

#ifdef NXPBUILD__PHAL_T1T_SW
extern phalT1T_Sw_DataParams_t            alT1T;              /* AL T1T component */
#endif /* NXPBUILD__PHAL_T1T_SW */

#ifdef NXPBUILD__PHAL_TOP_SW
extern phalTop_Sw_DataParams_t            salTop;             /* AL Tag Operations component */
#endif /* NXPBUILD__PHAL_TOP_SW */

#ifdef NXPBUILD__PHAL_I15693_SW
extern phalI15693_Sw_DataParams_t         salI15693;          /* AL ISO 15693 component */
#endif /* NXPBUILD__PHAL_I15693_SW */

#ifdef NXPBUILD__PHAL_SLI_SW
extern phalSli_Sw_DataParams_t            salSli;             /* AL Sli component */
#endif /* NXPBUILD__PHAL_SLI_SW */

#ifdef NXPBUILD__PHAL_I18000P3M3_SW
extern phalI18000p3m3_Sw_DataParams_t     salI18000p3m3;      /* AL ISO 18000p3m3 component */
#endif /* NXPBUILD__PHAL_I18000P3M3_SW */

#ifdef NXPBUILD__PHCE_T4T_SW
extern phceT4T_Sw_DataParams_t            sceT4T;             /* HCE component */
#endif /* NXPBUILD__PHCE_T4T_SW */

#ifdef NXPBUILD__PHLN_LLCP_SW
extern phlnLlcp_Sw_DataParams_t           slnLlcp;            /* LLCP component */
#endif /* NXPBUILD__PHLN_LLCP_SW */

#ifdef NXPBUILD__PH_CRYPTOSYM_SW
extern phCryptoSym_Sw_DataParams_t        sCryptoSym;         /* Crypto Sym component */
#endif /* NXPBUILD__PH_CRYPTOSYM_SW */

#ifdef NXPBUILD__PH_CRYPTORNG_SW
extern phCryptoRng_Sw_DataParams_t        sCryptoRng;         /* Crypto Rng component */
#endif /* NXPBUILD__PH_CRYPTORNG_SW */

//void                                      *pHal;              /* HAL pointer */

/**************************************************Prints if error is detected**************************************************************/
/* Enable(1) / Disable(0) printing error/info */
#define DETECT_ERROR 0

#if DETECT_ERROR
    #define DEBUG_ERROR_PRINT(x) x
    #define PRINT_INFO(...) DEBUG_PRINTF(__VA_ARGS__)
#else
    #define DEBUG_ERROR_PRINT(x)
    #define PRINT_INFO(...)
#endif

#define CHECK_STATUS(x)                                      \
    if ((x) != PH_ERR_SUCCESS)                               \
{                                                            \
    DEBUG_PRINTF("Line: %d   Error - (0x%04X) has occurred : 0xCCEE CC-Component ID, EE-Error code. Refer-ph_Status.h\n", __LINE__, (x));    \
}

/* prints if error is detected */
#define CHECK_SUCCESS(x)              \
    if ((x) != PH_ERR_SUCCESS)        \
{                                     \
    DEBUG_PRINTF("\nLine: %d   Error - (0x%04X) has occurred : 0xCCEE CC-Component ID, EE-Error code. Refer-ph_Status.h\n ", __LINE__, (x)); \
    return (x);                       \
}

/*********************************************************************************************************************************************/

/*******************************************************************************
**   Function Declarations
*******************************************************************************/
extern phStatus_t phApp_RdLibInit(void);
extern phStatus_t phApp_HALConfigAutoColl(void);
extern phStatus_t phApp_ConfigureLPCD(void);
extern void phApp_PrintTagInfo(phacDiscLoop_Sw_DataParams_t *pDataParams, uint16_t wNumberOfTags, uint16_t wTagsDetected);
extern void phApp_PrintTech(uint8_t TechType);
extern void phApp_Print_Buff(uint8_t *pBuff, uint8_t num);
extern void PrintErrorInfo(phStatus_t wStatus);

#endif /* PHAPP_INIT_H */
