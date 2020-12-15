#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/matt/ti/simplelink_cc2640r2_sdk_4_30_00_08/source;/Users/matt/ti/simplelink_cc2640r2_sdk_4_30_00_08/kernel/tirtos/packages;/Users/matt/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack;/Users/matt/ti/workspaces/ESLO_dev/ble5_simple_central_cc2640r2lp_app/.config
override XDCROOT = /Applications/ti/xdctools_3_51_03_28_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/matt/ti/simplelink_cc2640r2_sdk_4_30_00_08/source;/Users/matt/ti/simplelink_cc2640r2_sdk_4_30_00_08/kernel/tirtos/packages;/Users/matt/ti/simplelink_cc2640r2_sdk_4_30_00_08/source/ti/ble5stack;/Users/matt/ti/workspaces/ESLO_dev/ble5_simple_central_cc2640r2lp_app/.config;/Applications/ti/xdctools_3_51_03_28_core/packages;..
HOSTOS = MacOS
endif
