// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <asm/arch-fsl-layerscape/immap_lsch2.h>
#include <asm/arch-fsl-layerscape/fsl_icid.h>
#include <asm/arch-fsl-layerscape/fsl_portals.h>

#ifdef CONFIG_SYS_DPAA_QBMAN
struct qportal_info qp_info[CONFIG_SYS_QMAN_NUM_PORTALS] = {
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
	SET_QP_INFO(FSL_DPAA1_STREAM_ID_END, 0),
};
#endif

struct icid_id_table icid_tbl[] = {
#ifdef CONFIG_SYS_DPAA_QBMAN
	SET_QMAN_ICID(FSL_DPAA1_STREAM_ID_START),
	SET_BMAN_ICID(FSL_DPAA1_STREAM_ID_START + 1),
#endif

	SET_SDHC_ICID(FSL_SDHC_STREAM_ID),

	SET_USB_ICID(1, "snps,dwc3", FSL_USB1_STREAM_ID),
	SET_USB_ICID(2, "snps,dwc3", FSL_USB2_STREAM_ID),
	SET_USB_ICID(3, "snps,dwc3", FSL_USB3_STREAM_ID),

	SET_SATA_ICID("fsl,ls1046a-ahci", FSL_SATA_STREAM_ID),
	SET_QDMA_ICID("fsl,ls1046a-qdma", FSL_QDMA_STREAM_ID),
	SET_EDMA_ICID(FSL_EDMA_STREAM_ID),
	SET_ETR_ICID(FSL_ETR_STREAM_ID),
	SET_DEBUG_ICID(FSL_DEBUG_STREAM_ID),
#ifdef CONFIG_FSL_CAAM
	SET_SEC_QI_ICID(FSL_DPAA1_STREAM_ID_END),
	SET_SEC_JR_ICID_ENTRY(0, FSL_DPAA1_STREAM_ID_START + 3),
	SET_SEC_JR_ICID_ENTRY(1, FSL_DPAA1_STREAM_ID_START + 4),
	SET_SEC_JR_ICID_ENTRY(2, FSL_DPAA1_STREAM_ID_START + 5),
	SET_SEC_JR_ICID_ENTRY(3, FSL_DPAA1_STREAM_ID_START + 6),
	SET_SEC_RTIC_ICID_ENTRY(0, FSL_DPAA1_STREAM_ID_START + 7),
	SET_SEC_RTIC_ICID_ENTRY(1, FSL_DPAA1_STREAM_ID_START + 8),
	SET_SEC_RTIC_ICID_ENTRY(2, FSL_DPAA1_STREAM_ID_START + 9),
	SET_SEC_RTIC_ICID_ENTRY(3, FSL_DPAA1_STREAM_ID_START + 10),
	SET_SEC_DECO_ICID_ENTRY(0, FSL_DPAA1_STREAM_ID_START + 11),
	SET_SEC_DECO_ICID_ENTRY(1, FSL_DPAA1_STREAM_ID_START + 12),
	SET_SEC_DECO_ICID_ENTRY(2, FSL_DPAA1_STREAM_ID_START + 13),
#endif
};

int icid_tbl_sz = ARRAY_SIZE(icid_tbl);

#ifdef CONFIG_SYS_DPAA_FMAN
struct fman_icid_id_table fman_icid_tbl[] = {
	/* port id, icid */
	SET_FMAN_ICID_ENTRY(0x02, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x03, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x04, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x05, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x06, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x07, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x08, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x09, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x0a, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x0b, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x0c, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x0d, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x28, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x29, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x2a, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x2b, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x2c, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x2d, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x10, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x11, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x30, FSL_DPAA1_STREAM_ID_END),
	SET_FMAN_ICID_ENTRY(0x31, FSL_DPAA1_STREAM_ID_END),
};

int fman_icid_tbl_sz = ARRAY_SIZE(fman_icid_tbl);
#endif
