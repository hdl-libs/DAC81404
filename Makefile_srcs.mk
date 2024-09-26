
VVP_INCLUDEDIR=
VVP_INCLUDEDIR+=-I ./src
VVP_INCLUDEDIR+=-I ./sim

VVP_CFLAGS=
VVP_CFLAGS+=-g2005-sv

VVP_SRCS=

VVP_SRCS+= D:/ProgramFiles/modelsim_dlx64_10.6c/vivado2018.3_lib/sim_comm/apb_task.v
VVP_SRCS+= ./src/dac81404_conf.v
VVP_SRCS+= ./src/dac81404_conf_wrapper.v
VVP_SRCS+= ./src/dac81404_sacn.v
VVP_SRCS+= ./src/dac81404_sacn_wrapper.v
VVP_SRCS+= ./src/spi_master.v
VVP_SRCS+= ./src/round_arb.v
VVP_SRCS+= ./src/dac81404_ui.v
VVP_SRCS+= ./src/dac81404_wrapper.v

VVP_SRCS+= ./sim/dac81404_wrapper_tb.v