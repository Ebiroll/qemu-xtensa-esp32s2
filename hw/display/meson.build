hw_display_modules = {}

softmmu_ss.add(when: 'CONFIG_DDC', if_true: files('i2c-ddc.c'))
softmmu_ss.add(when: 'CONFIG_EDID', if_true: files('edid-generate.c', 'edid-region.c'))

softmmu_ss.add(when: 'CONFIG_FW_CFG_DMA', if_true: files('ramfb.c'))
softmmu_ss.add(when: 'CONFIG_FW_CFG_DMA', if_true: files('ramfb-standalone.c'))

softmmu_ss.add(when: 'CONFIG_ADS7846', if_true: files('ads7846.c'))
softmmu_ss.add(when: 'CONFIG_VGA_CIRRUS', if_true: files('cirrus_vga.c'))
softmmu_ss.add(when: ['CONFIG_VGA_CIRRUS', 'CONFIG_VGA_ISA'], if_true: files('cirrus_vga_isa.c'))
softmmu_ss.add(when: 'CONFIG_G364FB', if_true: files('g364fb.c'))
softmmu_ss.add(when: 'CONFIG_JAZZ_LED', if_true: files('jazz_led.c'))
softmmu_ss.add(when: 'CONFIG_PL110', if_true: files('pl110.c'))
softmmu_ss.add(when: 'CONFIG_SII9022', if_true: files('sii9022.c'))
softmmu_ss.add(when: 'CONFIG_SSD0303', if_true: files('ssd0303.c'))
softmmu_ss.add(when: 'CONFIG_SSD1306', if_true: files('ssd1306.c'))
softmmu_ss.add(when: 'CONFIG_SSD0323', if_true: files('ssd0323.c'))
softmmu_ss.add(when: 'CONFIG_XEN', if_true: files('xenfb.c'))

softmmu_ss.add(when: 'CONFIG_VGA_PCI', if_true: files('vga-pci.c'))
softmmu_ss.add(when: 'CONFIG_VGA_ISA', if_true: files('vga-isa.c'))
softmmu_ss.add(when: 'CONFIG_VGA_ISA_MM', if_true: files('vga-isa-mm.c'))
softmmu_ss.add(when: 'CONFIG_VMWARE_VGA', if_true: files('vmware_vga.c'))
softmmu_ss.add(when: 'CONFIG_BOCHS_DISPLAY', if_true: files('bochs-display.c'))

softmmu_ss.add(when: 'CONFIG_BLIZZARD', if_true: files('blizzard.c'))
softmmu_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_fimd.c'))
softmmu_ss.add(when: 'CONFIG_FRAMEBUFFER', if_true: files('framebuffer.c'))
softmmu_ss.add(when: 'CONFIG_ZAURUS', if_true: files('tc6393xb.c'))

softmmu_ss.add(when: 'CONFIG_OMAP', if_true: files('omap_dss.c'))
softmmu_ss.add(when: 'CONFIG_PXA2XX', if_true: files('pxa2xx_lcd.c'))
softmmu_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_fb.c'))
softmmu_ss.add(when: 'CONFIG_SM501', if_true: files('sm501.c'))
softmmu_ss.add(when: 'CONFIG_TCX', if_true: files('tcx.c'))
softmmu_ss.add(when: 'CONFIG_CG3', if_true: files('cg3.c'))
softmmu_ss.add(when: 'CONFIG_MACFB', if_true: files('macfb.c'))
softmmu_ss.add(when: 'CONFIG_NEXTCUBE', if_true: files('next-fb.c'))

specific_ss.add(when: 'CONFIG_VGA', if_true: files('vga.c'))

if config_all_devices.has_key('CONFIG_QXL')
  qxl_ss = ss.source_set()
  qxl_ss.add(when: 'CONFIG_QXL', if_true: [files('qxl.c', 'qxl-logger.c', 'qxl-render.c'),
                                           pixman, spice])
  hw_display_modules += {'qxl': qxl_ss}
endif

softmmu_ss.add(when: 'CONFIG_DPCD', if_true: files('dpcd.c'))
softmmu_ss.add(when: 'CONFIG_XLNX_ZYNQMP_ARM', if_true: files('xlnx_dp.c'))

softmmu_ss.add(when: 'CONFIG_MILKYMIST', if_true: files('milkymist-vgafb.c'))
softmmu_ss.add(when: 'CONFIG_ARTIST', if_true: files('artist.c'))

softmmu_ss.add(when: [pixman, 'CONFIG_ATI_VGA'], if_true: files('ati.c', 'ati_2d.c', 'ati_dbg.c'))

if config_all_devices.has_key('CONFIG_VIRTIO_GPU')
  virtio_gpu_ss = ss.source_set()
  virtio_gpu_ss.add(when: 'CONFIG_VIRTIO_GPU',
                    if_true: [files('virtio-gpu-base.c', 'virtio-gpu.c'), pixman, virgl])
  virtio_gpu_ss.add(when: ['CONFIG_VIRTIO_GPU', 'CONFIG_VIRGL'],
                    if_true: [files('virtio-gpu-3d.c'), pixman, virgl])
  virtio_gpu_ss.add(when: 'CONFIG_VHOST_USER_GPU', if_true: files('vhost-user-gpu.c'))
  hw_display_modules += {'virtio-gpu': virtio_gpu_ss}
endif

if config_all_devices.has_key('CONFIG_VIRTIO_PCI')
  virtio_gpu_pci_ss = ss.source_set()
  virtio_gpu_pci_ss.add(when: ['CONFIG_VIRTIO_GPU', 'CONFIG_VIRTIO_PCI'],
                        if_true: [files('virtio-gpu-pci.c'), pixman])
  virtio_gpu_pci_ss.add(when: ['CONFIG_VHOST_USER_GPU', 'CONFIG_VIRTIO_PCI'],
                        if_true: files('vhost-user-gpu-pci.c'))
  hw_display_modules += {'virtio-gpu-pci': virtio_gpu_pci_ss}
endif

if config_all_devices.has_key('CONFIG_VIRTIO_VGA')
  virtio_vga_ss = ss.source_set()
  virtio_vga_ss.add(when: 'CONFIG_VIRTIO_VGA',
                    if_true: [files('virtio-vga.c'), pixman])
  virtio_vga_ss.add(when: 'CONFIG_VHOST_USER_VGA',
                    if_true: files('vhost-user-vga.c'))
  hw_display_modules += {'virtio-vga': virtio_vga_ss}
endif

specific_ss.add(when: [x11, opengl, 'CONFIG_MILKYMIST_TMU2'], if_true: files('milkymist-tmu2.c'))
specific_ss.add(when: 'CONFIG_OMAP', if_true: files('omap_lcdc.c'))

modules += { 'hw-display': hw_display_modules }
