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
* Example Source abstracting component data structure and code initialization and code specific to HW used in the examples
* This file shall be present in all examples. A customer does not need to touch/modify this file. This file
* purely depends on the phNxpBuild_Lpc.h or phNxpBuild_App.h
* The phAppInit.h externs the component data structures initialized here that is in turn included by the core examples.
* The core example shall not use any other variable defined here except the RdLib component data structures(as explained above)
* The RdLib component initialization requires some user defined data and function pointers.
* These are defined in the respective examples and externed here.
*
* Keystore and Crypto initialization needs to be handled by application.
*
* $Author: jenkins_ cm (nxp92197) $
* $Revision: 4184 $ (v4.040.05.011646)
* $Date: 2016-01-22 18:04:59 +0530 (Fri, 22 Jan 2016) $
*
* History:
* BK: Generated 25. Sept 2014
*
*/

/* Status header */
#include <ph_Status.h>

#include "phApp_Init.h"

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

#ifdef NXPBUILD__PHHAL_HW_PN5180
/* HAL specific headers */
#include <phhalHw_Pn5180_Instr.h>
#endif

/*******************************************************************************
**   Global Variable Declaration
*******************************************************************************/
#ifdef NXPBUILD__PHPAL_I14443P4MC_SW
extern void pWtoxCallBck(uint8_t bTimerId);
#endif /* NXPBUILD__PHPAL_I14443P4MC_SW */

#ifdef NXPBUILD__PHPAL_I18092MT_SW
extern void pRtoxCallBck(uint8_t bTimerId);
#endif /* NXPBUILD__PHPAL_I18092MT_SW */

#ifdef NXPBUILD__PHCE_T4T_SW
/**
 * Application buffer. Used in phceT4T_Init. Its needed for data exchange
 * between application thread and reader library thread. Refer phceT4T_Init in
 * phceT4T.h for more info.
 * */
extern uint8_t aAppHCEBuf[];
extern uint16_t wAppHCEBuffSize;
#endif /* NXPBUILD__PHCE_T4T_SW */

phPlatform_DataParams_t            sPlatform;          /**< Platform component holder */

/* PAL declarations */
#ifdef NXPBUILD__PHPAL_I14443P3A_SW
phpalI14443p3a_Sw_DataParams_t     spalI14443p3a;      /* PAL ISO I14443-A component */
#endif /* NXPBUILD__PHPAL_I14443P3A_SW */

#ifdef NXPBUILD__PHPAL_I14443P3B_SW
phpalI14443p3b_Sw_DataParams_t     spalI14443p3b;      /* PAL ISO I14443-B component */
#endif /* NXPBUILD__PHPAL_I14443P3B_SW */

#ifdef NXPBUILD__PHPAL_I14443P4A_SW
phpalI14443p4a_Sw_DataParams_t     spalI14443p4a;      /* PAL ISO I14443-4A component */
#endif /* NXPBUILD__PHPAL_I14443P4A_SW */

#ifdef NXPBUILD__PHPAL_I14443P4_SW
phpalI14443p4_Sw_DataParams_t      spalI14443p4;       /* PAL ISO I14443-4 component */
#endif /* NXPBUILD__PHPAL_I14443P4_SW */

#ifdef NXPBUILD__PHPAL_MIFARE_SW
phpalMifare_Sw_DataParams_t        spalMifare;         /* PAL Mifare component */
#endif /* NXPBUILD__PHPAL_MIFARE_SW */

#ifdef NXPBUILD__PHPAL_FELICA_SW
phpalFelica_Sw_DataParams_t        spalFelica;         /* PAL Felica component */
#endif /* NXPBUILD__PHPAL_FELICA_SW */

#ifdef NXPBUILD__PHPAL_SLI15693_SW
phpalSli15693_Sw_DataParams_t      spalSli15693;       /* PAL ISO 15693 component */
#endif /* NXPBUILD__PHPAL_SLI15693_SW */

#ifdef NXPBUILD__PHPAL_I18000P3M3_SW
phpalI18000p3m3_Sw_DataParams_t    spalI18000p3m3;     /* PAL ISO 18000p3m3 component */
#endif /* NXPBUILD__PHPAL_I18000P3M3_SW */

#ifdef NXPBUILD__PHPAL_EPCUID_SW
phpalEpcUid_Sw_DataParams_t        spalEpcUid;         /* PAL EPC UID component */
#endif /* NXPBUILD__PHPAL_EPCUID_SW */

#ifdef NXPBUILD__PHPAL_I18092MPI_SW
phpalI18092mPI_Sw_DataParams_t     spalI18092mPI;      /* PAL ISO 18092 Initiator component */
#endif /* NXPBUILD__PHPAL_I18092MPI_SW */

#ifdef NXPBUILD__PHPAL_I14443P4MC_SW
phpalI14443p4mC_Sw_DataParams_t    spalI14443p4mC;     /* PAL ISO 14443 MC Target component */
#endif /* NXPBUILD__PHPAL_I14443P4MC_SW */

#ifdef NXPBUILD__PHPAL_I18092MT_SW
phpalI18092mT_Sw_DataParams_t      spalI18092mT;       /* PAL ISO 18092 Target component */
#endif /* NXPBUILD__PHPAL_I18092MT_SW */

/* Discovery Loop variable */
#ifdef NXPBUILD__PHAC_DISCLOOP_SW
phacDiscLoop_Sw_DataParams_t       sDiscLoop;          /* Discovery loop component */
#endif /* NXPBUILD__PHAC_DISCLOOP_SW */

/* Application Layer declaration */
#ifdef NXPBUILD__PHAL_FELICA_SW
phalFelica_Sw_DataParams_t         salFelica;          /* AL Felica component */
#endif /* NXPBUILD__PHAL_FELICA_SW */

#ifdef NXPBUILD__PHAL_MFC_SW
phalMfc_Sw_DataParams_t            salMFC;             /* AL Mifare Classic component */
#endif /* NXPBUILD__PHAL_MFC_SW */

#ifdef NXPBUILD__PHAL_MFUL_SW
phalMful_Sw_DataParams_t           salMFUL;            /* AL Mifare Ultralight component */
#endif /* NXPBUILD__PHAL_MFUL_SW */

#ifdef NXPBUILD__PHAL_MFDF_SW
phalMfdf_Sw_DataParams_t           salMFDF;            /* AL Mifare DESFire component */
#endif /* NXPBUILD__PHAL_MFDF_SW */

#ifdef NXPBUILD__PHAL_T1T_SW
phalT1T_Sw_DataParams_t            salT1T;             /* AL T1T component */
#endif /* NXPBUILD__PHAL_T1T_SW */

#ifdef NXPBUILD__PHAL_TOP_SW
phalTop_Sw_DataParams_t            salTop;             /* AL Tag Operations component */
#endif /* NXPBUILD__PHAL_TOP_SW */

#ifdef NXPBUILD__PHAL_I15693_SW
phalI15693_Sw_DataParams_t         salI15693;          /* AL ISO 15693 component */
#endif /* NXPBUILD__PHAL_I15693_SW */

#ifdef NXPBUILD__PHAL_SLI_SW
phalSli_Sw_DataParams_t            salSli;             /* AL Sli component */
#endif /* NXPBUILD__PHAL_SLI_SW */

#ifdef NXPBUILD__PHAL_I18000P3M3_SW
phalI18000p3m3_Sw_DataParams_t     salI18000p3m3;      /* AL ISO 18000p3m3 component */
#endif /* NXPBUILD__PHAL_I18000P3M3_SW */

#ifdef NXPBUILD__PHCE_T4T_SW
phceT4T_Sw_DataParams_t            sceT4T;             /* HCE component */
#endif /* NXPBUILD__PHCE_T4T_SW */

#ifdef NXPBUILD__PHLN_LLCP_SW
phlnLlcp_Sw_DataParams_t           slnLlcp;            /* LLCP component */
#endif /* NXPBUILD__PHLN_LLCP_SW */

#ifdef NXPBUILD__PH_CRYPTOSYM_SW
phCryptoSym_Sw_DataParams_t        sCryptoSym;         /* Crypto Sym component */
#endif /* NXPBUILD__PH_CRYPTOSYM_SW */

#ifdef NXPBUILD__PH_CRYPTORNG_SW
phCryptoRng_Sw_DataParams_t        sCryptoRng;         /* Crypto Rng component */
#endif /* NXPBUILD__PH_CRYPTORNG_SW */

/* General information bytes to be sent with ATR Request */
#if defined(NXPBUILD__PHPAL_I18092MPI_SW) || defined(NXPBUILD__PHPAL_I18092MT_SW)
uint8_t aLLCPGeneralBytes[36] = { 0x46,0x66,0x6D,
                                  0x01,0x01,0x10,       /*VERSION*/
                                  0x03,0x02,0x00,0x01,  /*WKS*/
                                  0x04,0x01,0xF1        /*LTO*/
                                 };
uint8_t   bLLCPGBLength = 13;
#endif

/* ATR Response or ATS Response holder */
#if defined(NXPBUILD__PHPAL_I14443P4A_SW)     || \
    defined(NXPBUILD__PHPAL_I18092MPI_SW)
uint8_t    aResponseHolder[64];
#endif

#ifdef NXPBUILD__PHHAL_HW_TARGET
/* Parameters for L3 activation during Autocoll */
extern uint8_t  sens_res[2]    ;
extern uint8_t  nfc_id1[3]     ;
extern uint8_t  sel_res        ;
extern uint8_t  nfc_id3        ;
extern uint8_t  poll_res[18]   ;
#endif /* NXPBUILD__PHHAL_HW_TARGET */

/* prints if error is detected */
#define CHECK_SUCCESS(x)              \
    if ((x) != PH_ERR_SUCCESS)        \
{                                     \
    DEBUG_PRINTF("\nLine: %d   Error - (0x%04X) has occurred : 0xCCEE CC-Component ID, EE-Error code. Refer-ph_Status.h\n ", __LINE__, (x)); \
    return (x);                       \
}

/**
* This function will initialize Reader LIbrary Component
*/
phStatus_t phApp_RdLibInit(void)
{
    phStatus_t wStatus;

#ifdef NXPBUILD__PHHAL_HW_RC663
#ifndef NXPBUILD__PH_RASPBERRY_PI
    /*
     * Set irq0en (08h) register bit 8 to 1 to configure interrupt as active low
     * User may change this to active high based on the platform.
     * */
    wStatus = phhalHw_Rc663_WriteRegister(&sPlatform.sHal, PHHAL_HW_RC663_REG_IRQ0EN, 0x80);
    CHECK_SUCCESS(wStatus);
#endif
#endif

/* Initialize the I14443-A PAL layer */
#ifdef NXPBUILD__PHPAL_I14443P3A_SW
    wStatus = phpalI14443p3a_Sw_Init(
        &spalI14443p3a,
        sizeof(phpalI14443p3a_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I14443P3A_SW */

/* Initialize the I14443-B PAL  component */
#ifdef NXPBUILD__PHPAL_I14443P3B_SW
    wStatus = phpalI14443p3b_Sw_Init(
        &spalI14443p3b,
        sizeof(spalI14443p3b),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I14443P3B_SW */

/* Initialize the I14443-A PAL component */
#ifdef NXPBUILD__PHPAL_I14443P4A_SW
    wStatus = phpalI14443p4a_Sw_Init(
        &spalI14443p4a,
        sizeof(phpalI14443p4a_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I14443P4A_SW */

/* Initialize the I14443-4 PAL component */
#ifdef NXPBUILD__PHPAL_I14443P4_SW
    wStatus = phpalI14443p4_Sw_Init(
        &spalI14443p4,
        sizeof(phpalI14443p4_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I14443P4_SW */

/* Initialize the Mifare PAL component */
#ifdef NXPBUILD__PHPAL_MIFARE_SW
    wStatus = phpalMifare_Sw_Init(
        &spalMifare,
        sizeof(phpalMifare_Sw_DataParams_t),
        &sPlatform.sHal,
#ifdef NXPBUILD__PHPAL_I14443P4_SW
        &spalI14443p4
#else
        NULL
#endif /* NXPBUILD__PHPAL_I14443P4_SW */
        );
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_MIFARE_SW */

/* Initialize PAL Felica PAL component */
#ifdef NXPBUILD__PHPAL_FELICA_SW
    wStatus = phpalFelica_Sw_Init(
        &spalFelica,
        sizeof(phpalFelica_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_FELICA_SW */

/* Initialize the 15693 PAL component */
#ifdef NXPBUILD__PHPAL_SLI15693_SW
    wStatus = phpalSli15693_Sw_Init(
        &spalSli15693,
        sizeof(phpalSli15693_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_SLI15693_SW */

/* Initialize the 1800p3m3 PAL component */
#ifdef NXPBUILD__PHPAL_I18000P3M3_SW
    wStatus = phpalI18000p3m3_Sw_Init(
        &spalI18000p3m3,
        sizeof(phpalI18000p3m3_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I18000P3M3_SW */

    /* Initialize EPC UID component */
#ifdef NXPBUILD__PHPAL_EPCUID_SW
    wStatus = phpalEpcUid_Sw_Init(
        &spalEpcUid,
        sizeof(phpalEpcUid_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_EPCUID_SW */

/* Initialize 18092 Initiator PAL component */
#ifdef NXPBUILD__PHPAL_I18092MPI_SW
    wStatus = phpalI18092mPI_Sw_Init(
        &spalI18092mPI,
        sizeof(phpalI18092mPI_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I18092MPI_SW */

/* Initialize 14443-4mC Target PAL component */
#ifdef NXPBUILD__PHPAL_I14443P4MC_SW
    wStatus = phpalI14443p4mC_Sw_Init(
        &spalI14443p4mC,
        sizeof(phpalI14443p4mC_Sw_DataParams_t),
        &sPlatform.sHal,
        &pWtoxCallBck);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I14443P4MC_SW */

/* Initialize 18092 Target PAL component */
#ifdef NXPBUILD__PHPAL_I18092MT_SW
    wStatus = phpalI18092mT_Sw_Init(
        &spalI18092mT,
        sizeof(phpalI18092mT_Sw_DataParams_t),
        &sPlatform.sHal,
        &pRtoxCallBck);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHPAL_I18092MT_SW */

/* Initialize AL Felica component */
#ifdef NXPBUILD__PHAL_FELICA_SW
    wStatus = phalFelica_Sw_Init(
        &salFelica,
        sizeof(phalFelica_Sw_DataParams_t),
        &spalFelica);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHAL_FELICA_SW */

/* Initialize AL Mifare Classic component */
#ifdef NXPBUILD__PHAL_MFC_SW
    wStatus = phalMfc_Sw_Init(
        &salMFC,
        sizeof(phalMfc_Sw_DataParams_t),
        &spalMifare,
#ifdef NXPBUILD__PH_KEYSTORE_SW
        &sPlatform.sKeyStore
#else
        NULL
#endif
        );
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHAL_MFC_SW */

/* Initialize AL Mifare Ultralight component */
#ifdef NXPBUILD__PHAL_MFUL_SW
    wStatus = phalMful_Sw_Init(
        &salMFUL,
        sizeof(phalMful_Sw_DataParams_t),
        &spalMifare,
#ifdef NXPBUILD__PH_KEYSTORE_SW
        &sPlatform.sKeyStore,
#else
        NULL,
#endif
#ifdef NXPBUILD__PH_CRYPTOSYM_SW
        &sCryptoSym,
#else
        NULL,
#endif
#ifdef NXPBUILD__PH_CRYPTORNG_SW
        &sCryptoRng
#else
        NULL
#endif
        );
#endif /* NXPBUILD__PHAL_MFUL_SW */

/* Initialize AL Mifare DESFire component */
#ifdef NXPBUILD__PHAL_MFDF_SW
    wStatus = phalMfdf_Sw_Init(
        &salMFDF,
        sizeof(phalMfdf_Sw_DataParams_t),
        &spalMifare,
#ifdef NXPBUILD__PH_KEYSTORE_SW
        &sPlatform.sKeyStore,
#else
        NULL,
#endif
#ifdef NXPBUILD__PH_CRYPTOSYM_SW
        &sCryptoSym,
#else
        NULL,
#endif
#ifdef NXPBUILD__PH_CRYPTORNG_SW
        &sCryptoRng,
#else
        NULL,
#endif
        &sPlatform.sHal
        );
#endif /* NXPBUILD__PHAL_MFDF_SW */

/* Initialize the T1T AL component */
#ifdef NXPBUILD__PHAL_T1T_SW
    wStatus = phalT1T_Sw_Init(
        &salT1T,
        sizeof(phalT1T_Sw_DataParams_t),
        &spalI14443p3a);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHAL_T1T_SW */

/* Initialize the ISO 15693 AL component */
#ifdef NXPBUILD__PHAL_I15693_SW
    wStatus = phalI15693_Sw_Init(
        &salI15693,
        sizeof(phalI15693_Sw_DataParams_t),
        &spalSli15693);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHAL_I15693_SW */

/* Initialize the Tag operations component */
#ifdef NXPBUILD__PHAL_TOP_SW
    wStatus = phalTop_Sw_Init(
        &salTop,
        sizeof(phalTop_Sw_DataParams_t),
#ifdef NXPBUILD__PHAL_T1T_SW
        &salT1T,
#else
        NULL,
#endif
#ifdef NXPBUILD__PHAL_MFUL_SW
        &salMFUL,
#else
        NULL,
#endif
#ifdef NXPBUILD__PHAL_FELICA_SW
        &salFelica,
#else
        NULL,
#endif
#ifdef NXPBUILD__PHAL_MFDF_SW
        &salMFDF,
#else
        NULL,
#endif
#ifdef NXPBUILD__PHAL_I15693_SW
        &salI15693
#else
        NULL
#endif
        );
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHAL_TOP_SW */

/* Initialize the Sli AL component */
#ifdef NXPBUILD__PHAL_SLI_SW
    wStatus = phalSli_Sw_Init(
        &salSli,
        sizeof(phalSli_Sw_DataParams_t),
        &spalSli15693);
#endif /* NXPBUILD__PHAL_SLI_SW */

/* Initialize the 18000p3m3 AL component */
#ifdef NXPBUILD__PHAL_I18000P3M3_SW
    wStatus = phalI18000p3m3_Sw_Init(
        &salI18000p3m3,
        sizeof(phalI18000p3m3_Sw_DataParams_t),
        &spalI18000p3m3);
    CHECK_SUCCESS(wStatus);
#endif /* NXPBUILD__PHAL_I18000P3M3_SW */

/* Initialize the HCE component */
#ifdef NXPBUILD__PHCE_T4T_SW
    wStatus = phceT4T_Sw_Init(
        &sceT4T,
        sizeof(phceT4T_Sw_DataParams_t),
        &spalI14443p4mC,
        aAppHCEBuf,
        wAppHCEBuffSize);
#endif /* NXPBUILD__PHCE_T4T_SW */

/* Initialize the LLCP component */
#ifdef NXPBUILD__PHLN_LLCP_SW
    slnLlcp.sLocalLMParams.wMiu = 0x00; /* 128 bytes only */
    slnLlcp.sLocalLMParams.wWks = 0x11; /* SNEP & LLCP */
    slnLlcp.sLocalLMParams.bLto = 100; /* Maximum LTO */
    slnLlcp.sLocalLMParams.bOpt = 0x02;
    slnLlcp.sLocalLMParams.bAvailableTlv = PHLN_LLCP_TLV_MIUX_MASK | PHLN_LLCP_TLV_WKS_MASK |
        PHLN_LLCP_TLV_LTO_MASK | PHLN_LLCP_TLV_OPT_MASK;

    wStatus = phlnLlcp_Sw_Init(
        &slnLlcp,
        sizeof(phlnLlcp_Sw_DataParams_t),
        aLLCPGeneralBytes,
        &bLLCPGBLength);
#endif /* NXPBUILD__PHLN_LLCP_SW */

/* Initialize the discover component */
#ifdef NXPBUILD__PHAC_DISCLOOP_SW
    wStatus = phacDiscLoop_Sw_Init(
        &sDiscLoop,
        sizeof(phacDiscLoop_Sw_DataParams_t),
        &sPlatform.sHal);
    CHECK_SUCCESS(wStatus);

    /* Assign other layer parameters in discovery loop */
    sDiscLoop.pHalDataParams          = &sPlatform.sHal;
#ifdef NXPBUILD__PHPAL_I14443P3A_SW
    sDiscLoop.pPal1443p3aDataParams   = &spalI14443p3a;
#endif /* NXPBUILD__PHPAL_I14443P3A_SW */

#ifdef NXPBUILD__PHPAL_I14443P3B_SW
    sDiscLoop.pPal1443p3bDataParams   = &spalI14443p3b;
#endif /* NXPBUILD__PHPAL_I14443P3B_SW */

#ifdef NXPBUILD__PHPAL_I14443P4A_SW
    sDiscLoop.pPal1443p4aDataParams   = &spalI14443p4a;
#endif /* NXPBUILD__PHPAL_I14443P4A_SW */

#ifdef NXPBUILD__PHPAL_I14443P4_SW
    sDiscLoop.pPal14443p4DataParams   = &spalI14443p4;
#endif /* NXPBUILD__PHPAL_I14443P4_SW */

#ifdef NXPBUILD__PHPAL_FELICA_SW
    sDiscLoop.pPalFelicaDataParams    = &spalFelica;
#endif /* NXPBUILD__PHPAL_FELICA_SW */

#ifdef NXPBUILD__PHPAL_SLI15693_SW
    sDiscLoop.pPalSli15693DataParams  = &spalSli15693;
#endif /* NXPBUILD__PHPAL_SLI15693_SW */

#ifdef NXPBUILD__PHPAL_I18092MPI_SW
    sDiscLoop.pPal18092mPIDataParams  = &spalI18092mPI;
#endif /* NXPBUILD__PHPAL_I18092MPI_SW */

#ifdef NXPBUILD__PHPAL_I18000P3M3_SW
    sDiscLoop.pPal18000p3m3DataParams = &spalI18000p3m3;
#endif /* NXPBUILD__PHPAL_I18000P3M3_SW */

#ifdef NXPBUILD__PHAL_I18000P3M3_SW
    sDiscLoop.pAl18000p3m3DataParams  = &salI18000p3m3;
#endif /* NXPBUILD__PHAL_I18000P3M3_SW */

#ifdef NXPBUILD__PHAL_T1T_SW
    sDiscLoop.pAlT1TDataParams        = &salT1T;
#endif /* NXPBUILD__PHAL_T1T_SW */

#if defined(NXPBUILD__PHPAL_I18092MPI_SW) || defined(NXPBUILD__PHPAL_I18092MT_SW)
    /* Assign the GI for Type A */
    sDiscLoop.sTypeATargetInfo.sTypeA_P2P.pGi       = (uint8_t *)aLLCPGeneralBytes;
    sDiscLoop.sTypeATargetInfo.sTypeA_P2P.bGiLength = bLLCPGBLength;
    /* Assign the GI for Type F */
    sDiscLoop.sTypeFTargetInfo.sTypeF_P2P.pGi       = (uint8_t *)aLLCPGeneralBytes;
    sDiscLoop.sTypeFTargetInfo.sTypeF_P2P.bGiLength = bLLCPGBLength;
#endif

#if defined(NXPBUILD__PHAC_DISCLOOP_TYPEA_P2P_TAGS) || defined(NXPBUILD__PHAC_DISCLOOP_TYPEA_P2P_ACTIVE)
    /* Assign ATR response for Type A */
    sDiscLoop.sTypeATargetInfo.sTypeA_P2P.pAtrRes   = aResponseHolder;
#endif
#if defined(NXPBUILD__PHAC_DISCLOOP_TYPEF_P2P_TAGS) ||  defined(NXPBUILD__PHAC_DISCLOOP_TYPEF212_P2P_ACTIVE) || defined(NXPBUILD__PHAC_DISCLOOP_TYPEF424_P2P_ACTIVE)
    /* Assign ATR response for Type F */
    sDiscLoop.sTypeFTargetInfo.sTypeF_P2P.pAtrRes   = aResponseHolder;
#endif
#ifdef NXPBUILD__PHAC_DISCLOOP_TYPEA_I3P4_TAGS
    /* Assign ATS buffer for Type A */
    sDiscLoop.sTypeATargetInfo.sTypeA_I3P4.pAts     = aResponseHolder;
#endif /* NXPBUILD__PHAC_DISCLOOP_TYPEA_I3P4_TAGS */
#endif /* NXPBUILD__PHAC_DISCLOOP_SW */
    return wStatus ;
}

/**
* This function will initialize Hal Target Config
*/
phStatus_t phApp_HALConfigAutoColl(void)
{
#ifdef NXPBUILD__PHHAL_HW_TARGET
    phStatus_t wStatus;
#endif /* NXPBUILD__PHHAL_HW_TARGET */

#if defined(NXPBUILD__PHHAL_HW_PN7462AU) && defined(NXPBUILD__PHHAL_HW_TARGET)
    uint8_t bSystemCode[] = {0xFF, 0xFF};
#endif /* NXPBUILD__PHHAL_HW_PN7462AU && NXPBUILD__PHHAL_HW_TARGET */

#if defined(NXPBUILD__PHHAL_HW_RC523) && defined(NXPBUILD__PHHAL_HW_TARGET)
    /* Set Listen Parameters in HAL Buffer used during Autocoll */
    wStatus = phhalHw_Rc523_SetListenParameters(
        &sPlatform.sHal,
        &sens_res[0],
        &nfc_id1[0],
        sel_res,
        &poll_res[0],
        nfc_id3);
    CHECK_SUCCESS(wStatus);
#endif

#if defined(NXPBUILD__PHHAL_HW_PN5180) && defined(NXPBUILD__PHHAL_HW_TARGET)
    /* Set Listen Parameters in HAL Buffer used during Autocoll */
    wStatus = phhalHw_Pn5180_SetListenParameters(
        &sPlatform.sHal,
        &sens_res[0],
        &nfc_id1[0],
        sel_res,
        &poll_res[0],
        nfc_id3);
    CHECK_SUCCESS(wStatus);
#endif

#if defined(NXPBUILD__PHHAL_HW_PN7462AU) && defined(NXPBUILD__PHHAL_HW_TARGET)
    /* Set Listen Parameters in HAL Buffer used during Autocoll */
    wStatus = phhalHw_PN7462AU_SetListenParam(
        &sPlatform.sHal,
       &sens_res[0],
       &nfc_id1[0],
       sel_res,
       (poll_res[0] == 0x02 ? PH_ON : PH_OFF),
       &poll_res[2],
       &poll_res[8],
       bSystemCode);
    CHECK_SUCCESS(wStatus);
#endif

    return PH_ERR_SUCCESS;
}

#ifdef NXPBUILD__PHHAL_HW_RC663
/* Configure LPCD (for CLRC663) */
phStatus_t phApp_ConfigureLPCD(void)
{
    phStatus_t status;
    uint8_t bValueI;
    uint8_t bValueQ;

    status = phhalHw_Rc663_Cmd_Lpcd_GetConfig(&sPlatform.sHal, &bValueI, &bValueQ);
    CHECK_SUCCESS(status);

    status = phhalHw_Rc663_Cmd_Lpcd_SetConfig(
        &sPlatform.sHal,
        PHHAL_HW_RC663_CMD_LPCD_MODE_POWERDOWN,
        bValueI,
        bValueQ,
        1,
        100);

    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_ENABLE_LPCD, PH_ON);
    CHECK_SUCCESS(status);

    return status;
}
#endif

#ifdef NXPBUILD__PHHAL_HW_PN5180
/* Configure LPCD (for PN5180) */
phStatus_t phApp_ConfigureLPCD(void)
{
    /**
     * PHHAL_HW_CONFIG_SET_LPCD_WAKEUPTIME_MS  0x0070U     //< Used value for wakeup counter in msecs, i.e. after this amount of time IC will wakes up from standby.
     * PHHAL_HW_CONFIG_LPCD_MODE               0x0071U     //< Used to set options  PHHAL_HW_PN5180_LPCD_MODE_DEFAULT or PHHAL_HW_PN5180_LPCD_MODE_POWERDOWN_GUARDED
     * PHHAL_HW_CONFIG_LPCD_REF                0x0072U     //< Used to set or get LPCD Ref
     */
        phStatus_t status;
        uint16_t wConfig = PHHAL_HW_CONFIG_LPCD_REF;
        uint16_t wValue;
        uint8_t bLPCD_Threshold_EEPROMAddress = 0x37;
        uint8_t bLPCD_Threshold = 0x10;
        wValue = PHHAL_HW_PN5180_LPCD_MODE_POWERDOWN;
        wConfig = PHHAL_HW_CONFIG_LPCD_MODE;

        //status = phhalHw_Pn5180_Int_LPCD_GetConfig(&sPlatform.sHal, wConfig, &wValue);
        status = phhalHw_Pn5180_Instr_WriteE2Prom(&sPlatform.sHal,bLPCD_Threshold_EEPROMAddress, &bLPCD_Threshold, 1 );
        CHECK_SUCCESS(status);
        status = phhalHw_Pn5180_Int_LPCD_SetConfig(
            &sPlatform.sHal,
            wConfig,
            wValue
            );

        status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_ENABLE_LPCD, PH_ON);
        CHECK_SUCCESS(status);

        return status;
}
#endif

#ifdef NXPBUILD__PHHAL_HW_PN7462AU
/* Configure LPCD (for PN7462AU) */
phStatus_t phApp_ConfigureLPCD(void)
{
    phStatus_t status;
    uint16_t wLpcdValue;

    wLpcdValue = 0;

    status = phhalHw_GetConfig(&sPlatform.sHal, PHHAL_HW_CONFIG_PN7462AU_LPCD_NEW_VALUE, &wLpcdValue);
    CHECK_SUCCESS(status);

    if (wLpcdValue != 0x00)
    {
        status = phhalHw_SetConfig(&sPlatform.sHal, PHHAL_HW_CONFIG_PN7462AU_LPCD_REF_VALUE, wLpcdValue);
        CHECK_SUCCESS(status);
    }

    status = phacDiscLoop_SetConfig(&sDiscLoop, PHAC_DISCLOOP_CONFIG_ENABLE_LPCD, PH_ON);
    CHECK_SUCCESS(status);

    return status;
}
#endif /* NXPBUILD__PHHAL_HW_PN7462AU */

/* Print technology being resolved */
void phApp_PrintTech(uint8_t TechType)
{
    switch(TechType)
    {
    case PHAC_DISCLOOP_POS_BIT_MASK_A:
        DEBUG_PRINTF ("\tResolving Type A... \n");
        break;

    case PHAC_DISCLOOP_POS_BIT_MASK_B:
        DEBUG_PRINTF ("\tResolving Type B... \n");
        break;

    case PHAC_DISCLOOP_POS_BIT_MASK_F212:
        DEBUG_PRINTF ("\tResolving Type F with baud rate 212... \n");
        break;

    case PHAC_DISCLOOP_POS_BIT_MASK_F424:
        DEBUG_PRINTF ("\tResolving Type F with baud rate 424... \n");
        break;

    case PHAC_DISCLOOP_POS_BIT_MASK_V:
        DEBUG_PRINTF ("\tResolving Type V... \n");
        break;

    default:
        break;
    }
}

/**
* This function will print buffer content
* \param   *pBuff   Buffer Reference
* \param   num      data size to be print
*/
void phApp_Print_Buff(uint8_t *pBuff, uint8_t num)
{
    uint32_t    i;

    for(i = 0; i < num; i++)
    {
        DEBUG_PRINTF(" %02X",pBuff[i]);
    }
}

/**
* This function will print Tag information
* \param   pDataParams      The discovery loop data parameters
* \param   wNumberOfTags    Total number of tags detected
* \param   wTagsDetected    Technology Detected
*/
void phApp_PrintTagInfo(phacDiscLoop_Sw_DataParams_t *pDataParams, uint16_t wNumberOfTags, uint16_t wTagsDetected)
{
    uint8_t bIndex;
    uint8_t bTagType;

#if defined(NXPBUILD__PHAC_DISCLOOP_TYPEA_TAGS) || defined(NXPBUILD__PHAC_DISCLOOP_TYPEA_P2P_ACTIVE)
    if (PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_A))
    {
        if(pDataParams->sTypeATargetInfo.bT1TFlag)
        {
            DEBUG_PRINTF("\tTechnology  : Type A");
            DEBUG_PRINTF ("\n\t\tUID :");
            phApp_Print_Buff( pDataParams->sTypeATargetInfo.aTypeA_I3P3[0].aUid,
                        pDataParams->sTypeATargetInfo.aTypeA_I3P3[0].bUidSize);
            DEBUG_PRINTF ("\n\t\tSAK : 0x%02x",pDataParams->sTypeATargetInfo.aTypeA_I3P3[0].aSak);
            DEBUG_PRINTF ("\n\t\tType: Type 1 Tag\n");
        }
        else
        {
            DEBUG_PRINTF("\tTechnology  : Type A");
            for(bIndex = 0; bIndex < wNumberOfTags; bIndex++)
            {
                DEBUG_PRINTF ("\n\t\tCard: %d",bIndex + 1);
                DEBUG_PRINTF ("\n\t\tUID :");
                phApp_Print_Buff( pDataParams->sTypeATargetInfo.aTypeA_I3P3[bIndex].aUid,
                            pDataParams->sTypeATargetInfo.aTypeA_I3P3[bIndex].bUidSize);
                DEBUG_PRINTF ("\n\t\tSAK : 0x%02x",pDataParams->sTypeATargetInfo.aTypeA_I3P3[bIndex].aSak);

                if ((pDataParams->sTypeATargetInfo.aTypeA_I3P3[bIndex].aSak & (uint8_t) ~0xFB) == 0)
                {
                    /* Bit b3 is set to zero, [Digital] 4.8.2 */
                    /* Mask out all other bits except for b7 and b6 */
                    bTagType = (pDataParams->sTypeATargetInfo.aTypeA_I3P3[bIndex].aSak & 0x60);
                    bTagType = bTagType >> 5;

                    switch(bTagType)
                    {
                    case PHAC_DISCLOOP_TYPEA_TYPE2_TAG_CONFIG_MASK:
                        DEBUG_PRINTF ("\n\t\tType: Type 2 Tag\n");
                        break;
                    case PHAC_DISCLOOP_TYPEA_TYPE4A_TAG_CONFIG_MASK:
                        DEBUG_PRINTF ("\n\t\tType: Type 4A Tag\n");
                        break;
                    case PHAC_DISCLOOP_TYPEA_TYPE_NFC_DEP_TAG_CONFIG_MASK:
                        DEBUG_PRINTF ("\n\t\tType: P2P\n");
                        break;
                    case PHAC_DISCLOOP_TYPEA_TYPE_NFC_DEP_TYPE4A_TAG_CONFIG_MASK:
                        DEBUG_PRINTF ("\n\t\tType: Type NFC_DEP and  4A Tag\n");
                        break;
                    default:
                        break;
                    }
                }
            }
        }
    }
#endif

#ifdef NXPBUILD__PHAC_DISCLOOP_TYPEB_TAGS
    if (PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_B))
    {
        DEBUG_PRINTF("\tTechnology  : Type B");
        /* Loop through all the Type B tags detected and print the Pupi */
        for (bIndex = 0; bIndex < wNumberOfTags; bIndex++)
        {
            DEBUG_PRINTF ("\n\t\tCard: %d",bIndex + 1);
            DEBUG_PRINTF ("\n\t\tUID :");
            /* PUPI Length is always 4 bytes */
            phApp_Print_Buff( pDataParams->sTypeBTargetInfo.aTypeB_I3P3[bIndex].aPupi, 0x04);
        }
        DEBUG_PRINTF("\n");
    }
#endif

#ifdef NXPBUILD__PHAC_DISCLOOP_TYPEF_TAGS
    if( PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_F212) ||
        PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_F424))
    {
        DEBUG_PRINTF("\tTechnology  : Type F");

        /* Loop through all the type F tags and print the IDm */
        for (bIndex = 0; bIndex < wNumberOfTags; bIndex++)
        {
            DEBUG_PRINTF ("\n\t\tCard: %d",bIndex + 1);
            DEBUG_PRINTF ("\n\t\tUID :");
            phApp_Print_Buff( pDataParams->sTypeFTargetInfo.aTypeFTag[bIndex].aIDmPMm,
                        PHAC_DISCLOOP_FELICA_IDM_LENGTH );
            if ((pDataParams->sTypeFTargetInfo.aTypeFTag[bIndex].aIDmPMm[0] == 0x01) &&
                (pDataParams->sTypeFTargetInfo.aTypeFTag[bIndex].aIDmPMm[1] == 0xFE))
            {
                /* This is Type F tag with P2P capabilities */
                DEBUG_PRINTF ("\n\t\tType: P2P");
            }
            else
            {
                /* This is Type F T3T tag */
                DEBUG_PRINTF ("\n\t\tType: Type 3 Tag");
            }

            if(pDataParams->sTypeFTargetInfo.aTypeFTag[bIndex].bBaud != PHAC_DISCLOOP_CON_BITR_212)
            {
                DEBUG_PRINTF ("\n\t\tBit Rate: 424\n");
            }
            else
            {
                DEBUG_PRINTF ("\n\t\tBit Rate: 212\n");
            }
        }
    }
#endif

#ifdef NXPBUILD__PHAC_DISCLOOP_TYPEV_TAGS
    if (PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_V))
    {
        DEBUG_PRINTF("\tTechnology  : Type V / ISO 15693 / T5T");
        /* Loop through all the Type V tags detected and print the UIDs */
        for (bIndex = 0; bIndex < wNumberOfTags; bIndex++)
        {
            DEBUG_PRINTF ("\n\t\tCard: %d",bIndex + 1);
            DEBUG_PRINTF ("\n\t\tUID :");
            phApp_Print_Buff( pDataParams->sTypeVTargetInfo.aTypeV[bIndex].aUid, 0x08);
        }
        DEBUG_PRINTF("\n");
    }
#endif

#ifdef NXPBUILD__PHAC_DISCLOOP_I18000P3M3_TAGS
    if (PHAC_DISCLOOP_CHECK_ANDMASK(wTagsDetected, PHAC_DISCLOOP_POS_BIT_MASK_18000P3M3))
    {
        DEBUG_PRINTF("\tTechnology  : ISO 18000p3m3 / EPC Gen2");
        /* Loop through all the 18000p3m3 tags detected and print the UII */
        for (bIndex = 0; bIndex < wNumberOfTags; bIndex++)
        {
            DEBUG_PRINTF("\n\t\tCard: %d",bIndex + 1);
            DEBUG_PRINTF("\n\t\tUII :");
            phApp_Print_Buff(
                pDataParams->sI18000p3m3TargetInfo.aI18000p3m3[bIndex].aUii,
                (pDataParams->sI18000p3m3TargetInfo.aI18000p3m3[bIndex].wUiiLength / 8));
        }
        DEBUG_PRINTF("\n");
    }
#endif
}

/**
* This function will print Error information received from Reader Lib
* \param   wStatus      Error status
*/
void PrintErrorInfo(phStatus_t wStatus)
{
    DEBUG_PRINTF("\n ErrorInfo Comp:");

    switch(wStatus & 0xFF00)
    {
    case PH_COMP_BAL:
        DEBUG_PRINTF("\t PH_COMP_BAL");
        break;
    case PH_COMP_HAL:
        DEBUG_PRINTF("\t PH_COMP_HAL");
        break;
    case PH_COMP_PAL_ISO14443P3A:
        DEBUG_PRINTF("\t PH_COMP_PAL_ISO14443P3A");
        break;
    case PH_COMP_PAL_ISO14443P3B:
        DEBUG_PRINTF("\t PH_COMP_PAL_ISO14443P3B");
        break;
    case PH_COMP_PAL_ISO14443P4A:
        DEBUG_PRINTF("\t PH_COMP_PAL_ISO14443P4A");
        break;
    case PH_COMP_PAL_ISO14443P4:
        DEBUG_PRINTF("\t PH_COMP_PAL_ISO14443P4");
        break;
    case PH_COMP_PAL_FELICA:
        DEBUG_PRINTF("\t PH_COMP_PAL_FELICA");
        break;
    case PH_COMP_PAL_EPCUID:
        DEBUG_PRINTF("\t PH_COMP_PAL_EPCUID");
        break;
    case PH_COMP_PAL_SLI15693:
        DEBUG_PRINTF("\t PH_COMP_PAL_SLI15693");
        break;
    case PH_COMP_PAL_I18000P3M3:
        DEBUG_PRINTF("\t PH_COMP_PAL_I18000P3M3");
        break;
    case PH_COMP_PAL_I18092MPI:
        DEBUG_PRINTF("\t PH_COMP_PAL_I18092MPI");
        break;
    case PH_COMP_PAL_I18092MT:
        DEBUG_PRINTF("\t PH_COMP_PAL_I18092MT");
        break;
    case PH_COMP_PAL_GENERALTARGET:
        DEBUG_PRINTF("\t PH_COMP_PAL_GENERALTARGET");
        break;
    case PH_COMP_PAL_I14443P4MC:
        DEBUG_PRINTF("\t PH_COMP_PAL_I14443P4MC");
        break;
    case PH_COMP_AC_DISCLOOP:
        DEBUG_PRINTF("\t PH_COMP_AC_DISCLOOP");
        break;
    case PH_COMP_OSAL:
        DEBUG_PRINTF("\t PH_COMP_OSAL");
        break;
    default:
        DEBUG_PRINTF("\t 0x%x",(wStatus & PH_COMPID_MASK));
        break;
    }

    DEBUG_PRINTF("\t type:");

    switch(wStatus & PH_ERR_MASK)
    {
    case PH_ERR_SUCCESS_INCOMPLETE_BYTE:
        DEBUG_PRINTF("\t PH_ERR_SUCCESS_INCOMPLETE_BYTE");
        break;
    case PH_ERR_IO_TIMEOUT:
        DEBUG_PRINTF("\t PH_ERR_IO_TIMEOUT");
        break;
    case PH_ERR_INTEGRITY_ERROR:
        DEBUG_PRINTF("\t PH_ERR_INTEGRITY_ERROR");
        break;
    case PH_ERR_COLLISION_ERROR:
        DEBUG_PRINTF("\t PH_ERR_COLLISION_ERROR");
        break;
    case PH_ERR_BUFFER_OVERFLOW:
        DEBUG_PRINTF("\t PH_ERR_BUFFER_OVERFLOW");
        break;
    case PH_ERR_FRAMING_ERROR:
        DEBUG_PRINTF("\t PH_ERR_FRAMING_ERROR");
        break;
    case PH_ERR_PROTOCOL_ERROR:
        DEBUG_PRINTF("\t PH_ERR_PROTOCOL_ERROR");
        break;
    case PH_ERR_RF_ERROR:
        DEBUG_PRINTF("\t PH_ERR_RF_ERROR");
        break;
    case PH_ERR_EXT_RF_ERROR:
        DEBUG_PRINTF("\t PH_ERR_EXT_RF_ERROR");
        break;
    case PH_ERR_NOISE_ERROR:
        DEBUG_PRINTF("\t PH_ERR_NOISE_ERROR");
        break;
    case PH_ERR_ABORTED:
        DEBUG_PRINTF("\t PH_ERR_ABORTED");
        break;
    case PH_ERR_INTERNAL_ERROR:
        DEBUG_PRINTF("\t PH_ERR_INTERNAL_ERROR");
        break;
    case PH_ERR_INVALID_DATA_PARAMS:
        DEBUG_PRINTF("\t PH_ERR_INVALID_DATA_PARAMS");
        break;
    case PH_ERR_INVALID_PARAMETER:
        DEBUG_PRINTF("\t PH_ERR_INVALID_PARAMETER");
        break;
    case PH_ERR_PARAMETER_OVERFLOW:
        DEBUG_PRINTF("\t PH_ERR_PARAMETER_OVERFLOW");
        break;
    case PH_ERR_UNSUPPORTED_PARAMETER:
        DEBUG_PRINTF("\t PH_ERR_UNSUPPORTED_PARAMETER");
        break;
    case PH_ERR_OSAL_ERROR:
        DEBUG_PRINTF("\t PH_ERR_OSAL_ERROR");
        break;
    case PHAC_DISCLOOP_LPCD_NO_TECH_DETECTED:
        DEBUG_PRINTF("\t PHAC_DISCLOOP_LPCD_NO_TECH_DETECTED");
        break;
    case PHAC_DISCLOOP_COLLISION_PENDING:
        DEBUG_PRINTF("\t PHAC_DISCLOOP_COLLISION_PENDING");
        break;
    default:
        DEBUG_PRINTF("\t 0x%x",(wStatus & PH_ERR_MASK));
        break;
    }
}


/******************************************************************************
**                            End Of File
******************************************************************************/
