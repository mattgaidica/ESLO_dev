#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_40_04_04/source;/Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_40_04_04/kernel/tirtos/packages;/Users/matt/ti/workspaces/ESLO_dev/simple_peripheral_CC2652RB_LAUNCHXL_tirtos_ccs/.config
override XDCROOT = /Users/matt/ti/xdctools_3_62_00_08_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_40_04_04/source;/Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_40_04_04/kernel/tirtos/packages;/Users/matt/ti/workspaces/ESLO_dev/simple_peripheral_CC2652RB_LAUNCHXL_tirtos_ccs/.config;/Users/matt/ti/xdctools_3_62_00_08_core/packages;..
HOSTOS = MacOS
endif
