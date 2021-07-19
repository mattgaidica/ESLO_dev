## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,em4f linker.cmd package/cfg/simple_peripheral_app_pem4f.oem4f

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/simple_peripheral_app_pem4f.xdl
	$(SED) 's"^\"\(package/cfg/simple_peripheral_app_pem4fcfg.cmd\)\"$""\"/Users/matt/ti/workspaces/ESLO_dev/simple_peripheral_CC2652RX/.config/xconfig_simple_peripheral_app/\1\""' package/cfg/simple_peripheral_app_pem4f.xdl > $@
	-$(SETDATE) -r:max package/cfg/simple_peripheral_app_pem4f.h compiler.opt compiler.opt.defs
