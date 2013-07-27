obj-y	 += pm.o standby.o

KBUILD_CFLAGS += -I$(srctree)/arch/arm/mach-sun7i/pm/standby

$(obj)/standby.o: $(obj)/standby/standby.bin

$(obj)/standby/standby.bin: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/standby
