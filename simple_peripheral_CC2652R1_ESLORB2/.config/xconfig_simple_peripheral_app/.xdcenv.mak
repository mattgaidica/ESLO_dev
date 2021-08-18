#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/matt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/source;/Users/matt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/kernel/tirtos/packages;/Users/matt/ti/workspaces/ESLO_dev/simple_peripheral_CC2652R1_ESLORB2/.config
override XDCROOT = /Applications/ti/ccs1010/xdctools_3_62_01_15_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/matt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/source;/Users/matt/ti/simplelink_cc13x2_26x2_sdk_5_20_00_52/kernel/tirtos/packages;/Users/matt/ti/workspaces/ESLO_dev/simple_peripheral_CC2652R1_ESLORB2/.config;/Applications/ti/ccs1010/xdctools_3_62_01_15_core/packages;..
HOSTOS = MacOS
endif
