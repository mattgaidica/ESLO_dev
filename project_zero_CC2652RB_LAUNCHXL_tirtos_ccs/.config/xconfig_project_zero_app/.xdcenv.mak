#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_30_00_54/source;/Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_30_00_54/kernel/tirtos/packages;/Users/matt/ti/workspaces/ESLO_dev/project_zero_CC2652RB_LAUNCHXL_tirtos_ccs/.config
override XDCROOT = /Applications/ti/ccs1010/xdctools_3_61_02_27_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_30_00_54/source;/Users/matt/ti/simplelink_cc13x2_26x2_sdk_4_30_00_54/kernel/tirtos/packages;/Users/matt/ti/workspaces/ESLO_dev/project_zero_CC2652RB_LAUNCHXL_tirtos_ccs/.config;/Applications/ti/ccs1010/xdctools_3_61_02_27_core/packages;..
HOSTOS = MacOS
endif
