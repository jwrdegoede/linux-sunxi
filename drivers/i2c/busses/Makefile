# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the i2c bus drivers.
#

# ACPI drivers
obj-$(CONFIG_I2C_SCMI)		+= i2c-scmi.o

# Auxiliary I2C/SMBus modules
obj-$(CONFIG_I2C_CCGX_UCSI)	+= i2c-ccgx-ucsi.o

# PC SMBus host controller drivers
obj-$(CONFIG_I2C_ALI1535)	+= i2c-ali1535.o
obj-$(CONFIG_I2C_ALI1563)	+= i2c-ali1563.o
obj-$(CONFIG_I2C_ALI15X3)	+= i2c-ali15x3.o
obj-$(CONFIG_I2C_AMD756)	+= i2c-amd756.o
obj-$(CONFIG_I2C_AMD8111)	+= i2c-amd8111.o
obj-$(CONFIG_I2C_CHT_WC)	+= i2c-cht-wc.o
obj-$(CONFIG_I2C_I801)		+= i2c-i801.o
obj-$(CONFIG_I2C_ISCH)		+= i2c-isch.o
obj-$(CONFIG_I2C_ISMT)		+= i2c-ismt.o
obj-$(CONFIG_I2C_NFORCE2)	+= i2c-nforce2.o
obj-$(CONFIG_I2C_NVIDIA_GPU)	+= i2c-nvidia-gpu.o
obj-$(CONFIG_I2C_PIIX4)		+= i2c-piix4.o
obj-$(CONFIG_I2C_SIS5595)	+= i2c-sis5595.o
obj-$(CONFIG_I2C_SIS630)	+= i2c-sis630.o
obj-$(CONFIG_I2C_SIS96X)	+= i2c-sis96x.o
obj-$(CONFIG_I2C_VIA)		+= i2c-via.o
obj-$(CONFIG_I2C_VIAPRO)	+= i2c-viapro.o
obj-$(CONFIG_I2C_ZHAOXIN)	+= i2c-viai2c-zhaoxin.o i2c-viai2c-common.o

# Mac SMBus host controller drivers
obj-$(CONFIG_I2C_HYDRA)		+= i2c-hydra.o
obj-$(CONFIG_I2C_POWERMAC)	+= i2c-powermac.o

# Embedded system I2C/SMBus host controller drivers
obj-$(CONFIG_I2C_ALTERA)	+= i2c-altera.o
obj-$(CONFIG_I2C_AMD_MP2)	+= i2c-amd-mp2-pci.o i2c-amd-mp2-plat.o
obj-$(CONFIG_I2C_AMD_ASF)	+= i2c-amd-asf-plat.o
obj-$(CONFIG_I2C_ASPEED)	+= i2c-aspeed.o
obj-$(CONFIG_I2C_AT91)		+= i2c-at91.o
i2c-at91-y			:= i2c-at91-core.o i2c-at91-master.o
i2c-at91-$(CONFIG_I2C_AT91_SLAVE_EXPERIMENTAL)	+= i2c-at91-slave.o
obj-$(CONFIG_I2C_AU1550)	+= i2c-au1550.o
obj-$(CONFIG_I2C_AXXIA)		+= i2c-axxia.o
obj-$(CONFIG_I2C_BCM2835)	+= i2c-bcm2835.o
obj-$(CONFIG_I2C_BCM_IPROC)	+= i2c-bcm-iproc.o
obj-$(CONFIG_I2C_CADENCE)	+= i2c-cadence.o
obj-$(CONFIG_I2C_CBUS_GPIO)	+= i2c-cbus-gpio.o
obj-$(CONFIG_I2C_CGBC)		+= i2c-cgbc.o
obj-$(CONFIG_I2C_CPM)		+= i2c-cpm.o
obj-$(CONFIG_I2C_DAVINCI)	+= i2c-davinci.o
obj-$(CONFIG_I2C_DESIGNWARE_CORE)			+= i2c-designware-core.o
i2c-designware-core-y					:= i2c-designware-common.o
i2c-designware-core-y					+= i2c-designware-master.o
i2c-designware-core-$(CONFIG_I2C_DESIGNWARE_SLAVE) 	+= i2c-designware-slave.o
obj-$(CONFIG_I2C_DESIGNWARE_PLATFORM)			+= i2c-designware-platform.o
i2c-designware-platform-y 				:= i2c-designware-platdrv.o
i2c-designware-platform-$(CONFIG_I2C_DESIGNWARE_AMDPSP)	+= i2c-designware-amdpsp.o
i2c-designware-platform-$(CONFIG_I2C_DESIGNWARE_BAYTRAIL) += i2c-designware-baytrail.o
obj-$(CONFIG_I2C_DESIGNWARE_AMDISP) += i2c-designware-amdisp.o
obj-$(CONFIG_I2C_DESIGNWARE_PCI)			+= i2c-designware-pci.o
i2c-designware-pci-y					:= i2c-designware-pcidrv.o
obj-$(CONFIG_I2C_DIGICOLOR)	+= i2c-digicolor.o
obj-$(CONFIG_I2C_EG20T)		+= i2c-eg20t.o
obj-$(CONFIG_I2C_EMEV2)		+= i2c-emev2.o
obj-$(CONFIG_I2C_EXYNOS5)	+= i2c-exynos5.o
obj-$(CONFIG_I2C_GPIO)		+= i2c-gpio.o
obj-$(CONFIG_I2C_HIGHLANDER)	+= i2c-highlander.o
obj-$(CONFIG_I2C_HISI)		+= i2c-hisi.o
obj-$(CONFIG_I2C_HIX5HD2)	+= i2c-hix5hd2.o
obj-$(CONFIG_I2C_IBM_IIC)	+= i2c-ibm_iic.o
obj-$(CONFIG_I2C_IMG)		+= i2c-img-scb.o
obj-$(CONFIG_I2C_IMX)		+= i2c-imx.o
obj-$(CONFIG_I2C_IMX_LPI2C)	+= i2c-imx-lpi2c.o
obj-$(CONFIG_I2C_IOP3XX)	+= i2c-iop3xx.o
obj-$(CONFIG_I2C_JZ4780)	+= i2c-jz4780.o
obj-$(CONFIG_I2C_K1)		+= i2c-k1.o
obj-$(CONFIG_I2C_KEBA)		+= i2c-keba.o
obj-$(CONFIG_I2C_KEMPLD)	+= i2c-kempld.o
obj-$(CONFIG_I2C_LPC2K)		+= i2c-lpc2k.o
obj-$(CONFIG_I2C_LS2X)		+= i2c-ls2x.o
obj-$(CONFIG_I2C_MESON)		+= i2c-meson.o
obj-$(CONFIG_I2C_MICROCHIP_CORE)	+= i2c-microchip-corei2c.o
obj-$(CONFIG_I2C_MPC)		+= i2c-mpc.o
obj-$(CONFIG_I2C_MT65XX)	+= i2c-mt65xx.o
obj-$(CONFIG_I2C_MT7621)	+= i2c-mt7621.o
obj-$(CONFIG_I2C_MV64XXX)	+= i2c-mv64xxx.o
obj-$(CONFIG_I2C_MXS)		+= i2c-mxs.o
obj-$(CONFIG_I2C_NOMADIK)	+= i2c-nomadik.o
obj-$(CONFIG_I2C_NPCM)		+= i2c-npcm7xx.o
obj-$(CONFIG_I2C_OCORES)	+= i2c-ocores.o
obj-$(CONFIG_I2C_OMAP)		+= i2c-omap.o
obj-$(CONFIG_I2C_OWL)		+= i2c-owl.o
obj-$(CONFIG_I2C_PASEMI)	+= i2c-pasemi-core.o i2c-pasemi-pci.o
obj-$(CONFIG_I2C_APPLE)		+= i2c-pasemi-core.o i2c-pasemi-platform.o
obj-$(CONFIG_I2C_PCA_PLATFORM)	+= i2c-pca-platform.o
obj-$(CONFIG_I2C_PNX)		+= i2c-pnx.o
obj-$(CONFIG_I2C_PXA)		+= i2c-pxa.o
obj-$(CONFIG_I2C_PXA_PCI)	+= i2c-pxa-pci.o
obj-$(CONFIG_I2C_QCOM_CCI)	+= i2c-qcom-cci.o
obj-$(CONFIG_I2C_QCOM_GENI)	+= i2c-qcom-geni.o
obj-$(CONFIG_I2C_QUP)		+= i2c-qup.o
obj-$(CONFIG_I2C_RIIC)		+= i2c-riic.o
obj-$(CONFIG_I2C_RK3X)		+= i2c-rk3x.o
obj-$(CONFIG_I2C_RTL9300)	+= i2c-rtl9300.o
obj-$(CONFIG_I2C_RZV2M)		+= i2c-rzv2m.o
obj-$(CONFIG_I2C_S3C2410)	+= i2c-s3c2410.o
obj-$(CONFIG_I2C_SH7760)	+= i2c-sh7760.o
obj-$(CONFIG_I2C_SH_MOBILE)	+= i2c-sh_mobile.o
obj-$(CONFIG_I2C_SIMTEC)	+= i2c-simtec.o
obj-$(CONFIG_I2C_SPRD)		+= i2c-sprd.o
obj-$(CONFIG_I2C_ST)		+= i2c-st.o
obj-$(CONFIG_I2C_STM32F4)	+= i2c-stm32f4.o
obj-$(CONFIG_I2C_STM32F7)	+= i2c-stm32f7-drv.o
i2c-stm32f7-drv-y		:= i2c-stm32f7.o i2c-stm32.o
obj-$(CONFIG_I2C_SUN6I_P2WI)	+= i2c-sun6i-p2wi.o
obj-$(CONFIG_I2C_SYNQUACER)	+= i2c-synquacer.o
obj-$(CONFIG_I2C_TEGRA)		+= i2c-tegra.o
obj-$(CONFIG_I2C_TEGRA_BPMP)	+= i2c-tegra-bpmp.o
obj-$(CONFIG_I2C_UNIPHIER)	+= i2c-uniphier.o
obj-$(CONFIG_I2C_UNIPHIER_F)	+= i2c-uniphier-f.o
obj-$(CONFIG_I2C_VERSATILE)	+= i2c-versatile.o
obj-$(CONFIG_I2C_WMT)		+= i2c-viai2c-wmt.o i2c-viai2c-common.o
obj-$(CONFIG_I2C_OCTEON)	+= i2c-octeon.o
i2c-octeon-y			:= i2c-octeon-core.o i2c-octeon-platdrv.o
obj-$(CONFIG_I2C_THUNDERX)	+= i2c-thunderx.o
i2c-thunderx-y			:= i2c-octeon-core.o i2c-thunderx-pcidrv.o
obj-$(CONFIG_I2C_XILINX)	+= i2c-xiic.o
obj-$(CONFIG_I2C_XLP9XX)	+= i2c-xlp9xx.o
obj-$(CONFIG_I2C_RCAR)		+= i2c-rcar.o
obj-$(CONFIG_I2C_GXP)		+= i2c-gxp.o

# External I2C/SMBus adapter drivers
obj-$(CONFIG_I2C_DIOLAN_U2C)	+= i2c-diolan-u2c.o
obj-$(CONFIG_I2C_DLN2)		+= i2c-dln2.o
obj-$(CONFIG_I2C_LJCA)		+= i2c-ljca.o
obj-$(CONFIG_I2C_USBIO)		+= i2c-usbio.o
obj-$(CONFIG_I2C_CP2615) += i2c-cp2615.o
obj-$(CONFIG_I2C_PARPORT)	+= i2c-parport.o
obj-$(CONFIG_I2C_PCI1XXXX)	+= i2c-mchp-pci1xxxx.o
obj-$(CONFIG_I2C_ROBOTFUZZ_OSIF)	+= i2c-robotfuzz-osif.o
obj-$(CONFIG_I2C_TAOS_EVM)	+= i2c-taos-evm.o
obj-$(CONFIG_I2C_TINY_USB)	+= i2c-tiny-usb.o
obj-$(CONFIG_I2C_VIPERBOARD)	+= i2c-viperboard.o

# Other I2C/SMBus bus drivers
obj-$(CONFIG_I2C_ACORN)		+= i2c-acorn.o
obj-$(CONFIG_I2C_BCM_KONA)	+= i2c-bcm-kona.o
obj-$(CONFIG_I2C_BRCMSTB)	+= i2c-brcmstb.o
obj-$(CONFIG_I2C_CROS_EC_TUNNEL)	+= i2c-cros-ec-tunnel.o
obj-$(CONFIG_I2C_ELEKTOR)	+= i2c-elektor.o
obj-$(CONFIG_I2C_ICY)		+= i2c-icy.o
obj-$(CONFIG_I2C_MLXBF)		+= i2c-mlxbf.o
obj-$(CONFIG_I2C_MLXCPLD)	+= i2c-mlxcpld.o
obj-$(CONFIG_I2C_OPAL)		+= i2c-opal.o
obj-$(CONFIG_I2C_PCA_ISA)	+= i2c-pca-isa.o
obj-$(CONFIG_I2C_SIBYTE)	+= i2c-sibyte.o
obj-$(CONFIG_I2C_XGENE_SLIMPRO) += i2c-xgene-slimpro.o
obj-$(CONFIG_SCx200_ACB)	+= scx200_acb.o
obj-$(CONFIG_I2C_FSI)		+= i2c-fsi.o
obj-$(CONFIG_I2C_VIRTIO)	+= i2c-virtio.o

ccflags-$(CONFIG_I2C_DEBUG_BUS) := -DDEBUG
