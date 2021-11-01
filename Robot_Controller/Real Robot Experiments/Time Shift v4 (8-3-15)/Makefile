
PRODUCT=xpc

 OUTDIR   := .
 MOD_TYPE := Mex

.PHONY: all
all : build

include $(BUILD_HARNESS)/makerules/arch.mk

 # This is shared code, and we won't build a mex file directly from them
 #
SKIP_MEX := canac2globals.c \
            evntcntrpci660xhelper.c \
            patibootcode.c \
            paticode.c \
            patisupport.c \
            pd2_dao.c \
            pd_caps.c \
            pd_hcaps.c \
            pdfw_lib.c \
            pdl_ain.c \
            pdl_aio.c \
            pdl_ao.c \
            pdl_brd.c \
            pdl_dio.c \
            pdl_evt.c \
            pdl_fwi.c \
            pdl_init.c \
            pdl_int.c \
            pdl_uct.c \
            quatechinit.c \
            xpcnitio.c \
            xpcionie.c \
            xpcionim.c \
            xpcsensoray.c \
            MexBFusrUtil.c \
            a2lgmxlexer.c \
            a2lgmxparser.c \
            mxparserhelpers.c \
            a2lparserhelpers.c


#ifeq ($(ARCH),win32)
#endif

#ifeq ($(ARCH),win64)
#endif

# Build on WIN32 and WIN64, not on Linux
ifeq ($(ARCH),win32)
 include $(MAKE_INCLUDE_DIR)/Makefile.module
else ifeq ($(ARCH),win64)
 include $(MAKE_INCLUDE_DIR)/Makefile.module
else
build:
	@echo No xPC Target MEX on Linux
	@echo $(ARCH)
endif

 A2LPARSER = $(SANDBOX_ROOT)/toolbox/rtw/targets/xpc/internal/target/build/drivers/XCP/a2lparser

 DEFSRCINC += -Iinclude \
              -I$(SANDBOX_ROOT)/msvc_headers \
              -I$(SANDBOX_ROOT)/rtw/c/src \
              -I$(SANDBOX_ROOT)/simulink/include \
              -I$(SANDBOX_ROOT)/src/include \
              -I$(SANDBOX_ROOT)/src/util/include \
              -I$(SANDBOX_ROOT)/toolbox/shared/testmeas/include \
              -I$(SANDBOX_ROOT)/toolbox/shared/can/src/scanutil \
			  -I$(A2LPARSER)/antlr-3.1.2/include \
              -I$(A2LPARSER)/include



# These mex files build using some shared code
# stitch up the shared objects with the mex files
$(OUTDIR)/xpcudpbytereceive$(MEX_EXT) : LIBDEPS += ws2_32.lib
$(OUTDIR)/xpcudpbytesend$(MEX_EXT) : LIBDEPS += ws2_32.lib

$(OUTDIR)/xpcimagetransmit$(MEX_EXT) : LIBDEPS += ws2_32.lib
$(OUTDIR)/xpcimagereceive$(MEX_EXT) : LIBDEPS += ws2_32.lib


$(OUTDIR)/canac2objreceive$(MEX_EXT) : LIBDEPS += scanutil.lib
$(OUTDIR)/canac2objsend$(MEX_EXT) : LIBDEPS += scanutil.lib
$(OUTDIR)/canac2fifosend$(MEX_EXT) : LIBDEPS += scanutil.lib
$(OUTDIR)/canac2fiforeceive$(MEX_EXT) : LIBDEPS += scanutil.lib
$(OUTDIR)/j1939protocolstack$(MEX_EXT) : LIBDEPS += scanutil.lib

$(OUTDIR)/xpcparsebfcam$(MEX_EXT) : $(OBJDIR)/MexBFusrUtil$(OBJ_EXT)

ifeq ($(ARCH),win32)
$(OUTDIR)/jpegcomp$(MEX_EXT) : LIBDEPS += lib/jpeg_vc.lib
$(OUTDIR)/jpegdecomp$(MEX_EXT) : LIBDEPS += lib/jpeg_vc.lib
LIBDEPS += $(A2LPARSER)/antlr-3.1.2/lib/antlr3c.lib
endif

ifeq ($(ARCH),win64)
$(OUTDIR)/jpegcomp$(MEX_EXT) : LIBDEPS += lib/jpeg_vc64.lib
$(OUTDIR)/jpegdecomp$(MEX_EXT) : LIBDEPS += lib/jpeg_vc64.lib
LIBDEPS += $(A2LPARSER)/antlr-3.1.2/lib/x64/antlr3c.lib
endif 

$(OUTDIR)/mx_a2l_gmx$(MEX_EXT) : $(OBJDIR)/a2lgmxparser$(OBJ_EXT) $(OBJDIR)/a2lgmxlexer$(OBJ_EXT) $(OBJDIR)/a2lparserhelpers$(OBJ_EXT) $(OBJDIR)/mxparserhelpers$(OBJ_EXT)

