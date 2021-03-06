#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-PIC32MX795.mk)" "nbproject/Makefile-local-PIC32MX795.mk"
include nbproject/Makefile-local-PIC32MX795.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PIC32MX795
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../../../../bno055.c ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c ../src/app.c ../src/UART.c ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_static_buffer_model.c ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_mapping.c ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static.c ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/PIC32MX795/framework/system/clk/src/sys_clk_pic32mx.c ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon.c ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon_pic32mx.c ../src/system_config/PIC32MX795/framework/system/ports/src/sys_ports_static.c ../src/system_config/PIC32MX795/bsp/bsp.c ../src/system_config/PIC32MX795/system_init.c ../src/system_config/PIC32MX795/system_interrupt.c ../src/system_config/PIC32MX795/system_exceptions.c ../src/main.c ../src/system_config/PIC32MX795/system_tasks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1568745167/bno055.o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/UART.o ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o ${OBJECTDIR}/_ext/788194979/drv_usart_static.o ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o ${OBJECTDIR}/_ext/551892840/sys_devcon.o ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o ${OBJECTDIR}/_ext/943318950/bsp.o ${OBJECTDIR}/_ext/88307446/system_init.o ${OBJECTDIR}/_ext/88307446/system_interrupt.o ${OBJECTDIR}/_ext/88307446/system_exceptions.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/88307446/system_tasks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1568745167/bno055.o.d ${OBJECTDIR}/_ext/709153290/drv_tmr.o.d ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d ${OBJECTDIR}/_ext/910924237/sys_tmr.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/UART.o.d ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d ${OBJECTDIR}/_ext/551892840/sys_devcon.o.d ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d ${OBJECTDIR}/_ext/943318950/bsp.o.d ${OBJECTDIR}/_ext/88307446/system_init.o.d ${OBJECTDIR}/_ext/88307446/system_interrupt.o.d ${OBJECTDIR}/_ext/88307446/system_exceptions.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/88307446/system_tasks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1568745167/bno055.o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/UART.o ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o ${OBJECTDIR}/_ext/788194979/drv_usart_static.o ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o ${OBJECTDIR}/_ext/551892840/sys_devcon.o ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o ${OBJECTDIR}/_ext/943318950/bsp.o ${OBJECTDIR}/_ext/88307446/system_init.o ${OBJECTDIR}/_ext/88307446/system_interrupt.o ${OBJECTDIR}/_ext/88307446/system_exceptions.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/88307446/system_tasks.o

# Source Files
SOURCEFILES=../../../../bno055.c ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c ../src/app.c ../src/UART.c ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_static_buffer_model.c ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_mapping.c ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static.c ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/PIC32MX795/framework/system/clk/src/sys_clk_pic32mx.c ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon.c ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon_pic32mx.c ../src/system_config/PIC32MX795/framework/system/ports/src/sys_ports_static.c ../src/system_config/PIC32MX795/bsp/bsp.c ../src/system_config/PIC32MX795/system_init.c ../src/system_config/PIC32MX795/system_interrupt.c ../src/system_config/PIC32MX795/system_exceptions.c ../src/main.c ../src/system_config/PIC32MX795/system_tasks.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-PIC32MX795.mk dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1568745167/bno055.o: ../../../../bno055.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1568745167" 
	@${RM} ${OBJECTDIR}/_ext/1568745167/bno055.o.d 
	@${RM} ${OBJECTDIR}/_ext/1568745167/bno055.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1568745167/bno055.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1568745167/bno055.o.d" -o ${OBJECTDIR}/_ext/1568745167/bno055.o ../../../../bno055.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/709153290/drv_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/709153290" 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/231205469/sys_int_pic32.o: ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/231205469" 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/910924237/sys_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/910924237" 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/UART.o: ../src/UART.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/UART.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/UART.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/UART.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/UART.o.d" -o ${OBJECTDIR}/_ext/1360937237/UART.o ../src/UART.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o: ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_static_buffer_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/154000690" 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d" -o ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_static_buffer_model.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o: ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/154000690" 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d" -o ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_mapping.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o: ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/788194979" 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/788194979/drv_usart_static.o: ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/788194979" 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/788194979/drv_usart_static.o ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o: ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static_byte_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/788194979" 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o: ../src/system_config/PIC32MX795/framework/system/clk/src/sys_clk_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/781659329" 
	@${RM} ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d" -o ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o ../src/system_config/PIC32MX795/framework/system/clk/src/sys_clk_pic32mx.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/551892840/sys_devcon.o: ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/551892840" 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/551892840/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/551892840/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/551892840/sys_devcon.o ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o: ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/551892840" 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon_pic32mx.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1937243855/sys_ports_static.o: ../src/system_config/PIC32MX795/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1937243855" 
	@${RM} ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o ../src/system_config/PIC32MX795/framework/system/ports/src/sys_ports_static.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/943318950/bsp.o: ../src/system_config/PIC32MX795/bsp/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/943318950" 
	@${RM} ${OBJECTDIR}/_ext/943318950/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/943318950/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/943318950/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/943318950/bsp.o.d" -o ${OBJECTDIR}/_ext/943318950/bsp.o ../src/system_config/PIC32MX795/bsp/bsp.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_init.o: ../src/system_config/PIC32MX795/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_init.o.d" -o ${OBJECTDIR}/_ext/88307446/system_init.o ../src/system_config/PIC32MX795/system_init.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_interrupt.o: ../src/system_config/PIC32MX795/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/88307446/system_interrupt.o ../src/system_config/PIC32MX795/system_interrupt.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_exceptions.o: ../src/system_config/PIC32MX795/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/88307446/system_exceptions.o ../src/system_config/PIC32MX795/system_exceptions.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_tasks.o: ../src/system_config/PIC32MX795/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DICD4Tool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_tasks.o.d" -o ${OBJECTDIR}/_ext/88307446/system_tasks.o ../src/system_config/PIC32MX795/system_tasks.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/1568745167/bno055.o: ../../../../bno055.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1568745167" 
	@${RM} ${OBJECTDIR}/_ext/1568745167/bno055.o.d 
	@${RM} ${OBJECTDIR}/_ext/1568745167/bno055.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1568745167/bno055.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1568745167/bno055.o.d" -o ${OBJECTDIR}/_ext/1568745167/bno055.o ../../../../bno055.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/709153290/drv_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/709153290" 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/709153290/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/709153290/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/709153290/drv_tmr.o ../../../../../../microchip/harmony/v2_06/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/231205469/sys_int_pic32.o: ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/231205469" 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/231205469/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/231205469/sys_int_pic32.o ../../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/910924237/sys_tmr.o: ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/910924237" 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/910924237/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/910924237/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/910924237/sys_tmr.o ../../../../../../microchip/harmony/v2_06/framework/system/tmr/src/sys_tmr.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/UART.o: ../src/UART.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/UART.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/UART.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/UART.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/UART.o.d" -o ${OBJECTDIR}/_ext/1360937237/UART.o ../src/UART.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o: ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_static_buffer_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/154000690" 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o.d" -o ${OBJECTDIR}/_ext/154000690/drv_i2c_static_buffer_model.o ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_static_buffer_model.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o: ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/154000690" 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o.d" -o ${OBJECTDIR}/_ext/154000690/drv_i2c_mapping.o ../src/system_config/PIC32MX795/framework/driver/i2c/src/drv_i2c_mapping.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o: ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/788194979" 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/788194979/drv_usart_mapping.o ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/788194979/drv_usart_static.o: ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/788194979" 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/788194979/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/788194979/drv_usart_static.o ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o: ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static_byte_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/788194979" 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/788194979/drv_usart_static_byte_model.o ../src/system_config/PIC32MX795/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o: ../src/system_config/PIC32MX795/framework/system/clk/src/sys_clk_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/781659329" 
	@${RM} ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o.d" -o ${OBJECTDIR}/_ext/781659329/sys_clk_pic32mx.o ../src/system_config/PIC32MX795/framework/system/clk/src/sys_clk_pic32mx.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/551892840/sys_devcon.o: ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/551892840" 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/551892840/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/551892840/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/551892840/sys_devcon.o ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o: ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/551892840" 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/551892840/sys_devcon_pic32mx.o ../src/system_config/PIC32MX795/framework/system/devcon/src/sys_devcon_pic32mx.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1937243855/sys_ports_static.o: ../src/system_config/PIC32MX795/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1937243855" 
	@${RM} ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1937243855/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/1937243855/sys_ports_static.o ../src/system_config/PIC32MX795/framework/system/ports/src/sys_ports_static.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/943318950/bsp.o: ../src/system_config/PIC32MX795/bsp/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/943318950" 
	@${RM} ${OBJECTDIR}/_ext/943318950/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/943318950/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/943318950/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/943318950/bsp.o.d" -o ${OBJECTDIR}/_ext/943318950/bsp.o ../src/system_config/PIC32MX795/bsp/bsp.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_init.o: ../src/system_config/PIC32MX795/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_init.o.d" -o ${OBJECTDIR}/_ext/88307446/system_init.o ../src/system_config/PIC32MX795/system_init.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_interrupt.o: ../src/system_config/PIC32MX795/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/88307446/system_interrupt.o ../src/system_config/PIC32MX795/system_interrupt.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_exceptions.o: ../src/system_config/PIC32MX795/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/88307446/system_exceptions.o ../src/system_config/PIC32MX795/system_exceptions.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/88307446/system_tasks.o: ../src/system_config/PIC32MX795/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/88307446" 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/88307446/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/88307446/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../../../../../../../microchip/harmony/v2_06/framework" -I"../../../../../../../Users/juan.y/microchip/harmony/v2_06/framework" -I"../src" -I"../src/system_config/PIC32MX795" -I"../src/PIC32MX795" -I"../../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/PIC32MX795/framework" -I"../src/system_config/PIC32MX795/bsp" -MMD -MF "${OBJECTDIR}/_ext/88307446/system_tasks.o.d" -o ${OBJECTDIR}/_ext/88307446/system_tasks.o ../src/system_config/PIC32MX795/system_tasks.c    -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MX795F512L_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -DICD4Tool=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\microchip\harmony\v2_06\bin\framework\peripheral\PIC32MX795F512L_peripherals.a      -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=ICD4Tool=1,--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MX795F512L_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\microchip\harmony\v2_06\bin\framework\peripheral\PIC32MX795F512L_peripherals.a      -DXPRJ_PIC32MX795=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/BNO055_EXP16.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PIC32MX795
	${RM} -r dist/PIC32MX795

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
